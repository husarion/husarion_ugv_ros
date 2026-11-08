// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_DRIVER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_DRIVER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include "lely/coapp/loop_driver.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/cmd_write_status.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"

namespace husarion_ugv_hardware_interfaces
{

// Forward declaration
class RoboteqMotorDriver;

/**
 * @brief Hardware implementation of Driver with lely LoopDriver for Roboteq drivers
 * control
 */
// What SyncSDOWrite does with a write the master has not run when the caller stops waiting.
enum class SdoLateWrite {
  // Dropped: a release that goes out after the caller reported a failure must never happen.
  kDrop,
  // Sent when the master runs it: a latch that goes out late is still what the caller wanted.
  kKeepQueued,
};

class RoboteqDriver : public DriverInterface,
                      public lely::canopen::LoopDriver,
                      public std::enable_shared_from_this<RoboteqDriver>
{
public:
  /**
   * @brief Must be owned by a std::shared_ptr: SDO writes and velocity commands throw otherwise.
   */
  RoboteqDriver(
    const std::shared_ptr<lely::canopen::AsyncMaster> & async_master, const std::uint8_t id,
    const std::chrono::milliseconds & sdo_operation_timeout_ms);

  /**
   * @brief Triggers boot operations
   *
   * @exception std::runtime_error if triggering boot fails
   */
  std::future<void> Boot() override;

  /**
   * @brief Returns true if CAN error was detected.
   */
  bool IsCANError() const override { return can_error_.load(); };

  /**
   * @brief Returns true if heartbeat timeout encountered.
   */
  bool IsHeartbeatTimeout() const override { return heartbeat_timeout_.load(); };

  /**
   * @brief Reads driver state data returned from Roboteq (PDO 3 and 4): error flags, battery
   * voltage, battery currents (for channel 1 and 2, they are not the same as motor currents),
   * temperatures. Also saves the last timestamps
   */
  DriverState ReadState() override;

  /**
   * @exception std::runtime_error if any operation returns error
   */
  void ResetScript() override;

  /**
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnEStop() override;

  /**
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOffEStop() override;

  /**
   * @brief Adds a motor driver to the driver
   */
  void AddMotorDriver(
    const MotorNames name, std::shared_ptr<MotorDriverInterface> motor_driver) override;

  /**
   * @brief Returns a motor driver by name
   *
   * @exception std::runtime_error if motor driver with the given name does not exist
   */
  std::shared_ptr<MotorDriverInterface> GetMotorDriver(const MotorNames name) override;

  /**
   * @brief Blocking SDO write operation. The write is submitted on the CANopen master's executor
   * and the wait is bounded by the SDO timeout plus kSdoConfirmationTimeoutMargin.
   *
   * @param late_write what happens to a write the master has not run when the wait ends.
   * @exception std::runtime_error if operation fails or no confirmation arrives in time
   */
  template <typename T>
  void SyncSDOWrite(
    const std::uint16_t index, const std::uint8_t subindex, const T data,
    const SdoLateWrite late_write = SdoLateWrite::kDrop);

  /**
   * @brief Returns the last timestamp of the position data for the given channel
   */
  timespec GetPositionTimestamp(const std::uint8_t channel) const
  {
    return ToTimespec(
      last_position_timestamps_ns_.at(channel - kChannel1).load(std::memory_order_acquire));
  }

  /**
   * @brief Returns the last timestamp of the speed and current data for the given channel
   */
  timespec GetSpeedCurrentTimestamp(const std::uint8_t channel) const
  {
    return ToTimespec(
      last_speed_current_timestamps_ns_.at(channel - kChannel1).load(std::memory_order_acquire));
  }

  /**
   * @brief Returns the last received motor position, velocity or current for the given channel
   */
  std::int32_t GetPosition(const std::uint8_t channel) const
  {
    return last_positions_.at(channel - kChannel1).load(std::memory_order_acquire);
  }

  std::int16_t GetVelocity(const std::uint8_t channel) const
  {
    return last_velocities_.at(channel - kChannel1).load(std::memory_order_acquire);
  }

  std::int16_t GetCurrent(const std::uint8_t channel) const
  {
    return last_currents_.at(channel - kChannel1).load(std::memory_order_acquire);
  }

  /**
   * @brief Hands a velocity command to the CANopen master's executor, which writes the mapped
   * TPDO object. Only the newest command per channel is written.
   *
   * @exception std::runtime_error if the newest completed write on this channel failed
   */
  void PostCmdVel(const std::uint8_t channel, const std::int32_t cmd);

  static constexpr std::uint8_t kChannel1 = 1;
  static constexpr std::uint8_t kChannel2 = 2;

private:
  // Tasks posted to the master's executor hold this, not a raw pointer, so a task that runs or is
  // dropped after the driver is gone does nothing.
  std::weak_ptr<RoboteqDriver> GetWeakSelf();
  void PostCmdVelTask(const std::uint8_t channel, const std::size_t slot);

  static std::int64_t ToNanoseconds(const timespec & ts)
  {
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL +
           static_cast<std::int64_t>(ts.tv_nsec);
  }

  static timespec ToTimespec(const std::int64_t ns)
  {
    timespec ts;
    ts.tv_sec = static_cast<std::time_t>(ns / 1000000000LL);
    ts.tv_nsec = static_cast<long>(ns % 1000000000LL);
    return ts;
  }

  void OnBoot(
    const lely::canopen::NmtState st, const char es, const std::string & what) noexcept override;
  void OnRpdoWrite(const std::uint16_t idx, const std::uint8_t subidx) noexcept override;
  void OnCanError(const lely::io::CanError /* error */) noexcept override
  {
    can_error_.store(true);
  }

  void OnHeartbeat(const bool occurred) noexcept override { heartbeat_timeout_.store(occurred); }

  std::mutex boot_mtx_;
  std::promise<void> boot_promise_;

  std::atomic_bool can_error_;
  std::atomic_bool heartbeat_timeout_;

  // Written by OnRpdoWrite on this driver's loop thread and read by the control loop without the
  // master lock that rpdo_mapped takes. Indexed by channel - kChannel1.
  std::array<std::atomic<std::int64_t>, 2> last_position_timestamps_ns_{};
  std::array<std::atomic<std::int64_t>, 2> last_speed_current_timestamps_ns_{};
  std::atomic<std::int64_t> flags_current_timestamp_ns_{0};
  std::atomic<std::int64_t> last_voltages_temps_timestamp_ns_{0};

  std::array<std::atomic<std::int32_t>, 2> last_positions_{};
  std::array<std::atomic<std::int16_t>, 2> last_velocities_{};
  std::array<std::atomic<std::int16_t>, 2> last_currents_{};
  std::atomic<std::int32_t> last_flags_{0};
  std::atomic<std::int16_t> last_mcu_temp_{0};
  std::atomic<std::int16_t> last_heatsink_temp_{0};
  std::atomic<std::uint16_t> last_battery_voltage_{0};
  std::atomic<std::int16_t> last_battery_current_1_{0};
  std::atomic<std::int16_t> last_battery_current_2_{0};

  // One queued task per channel, which writes the newest command when it runs.
  std::array<std::atomic<std::int32_t>, 2> pending_cmd_{};
  std::array<std::atomic_bool, 2> cmd_task_queued_{};
  // Steady clock time the queued task was posted, written by the control loop only.
  std::array<std::atomic<std::chrono::steady_clock::rep>, 2> cmd_task_posted_{};
  // About ten control cycles: a master this late is not running the tasks at all.
  static constexpr std::chrono::milliseconds kCmdTaskDeadline{100};

  CmdWriteStatus cmd_write_status_;

  std::atomic_bool rpdo_read_error_logged_{false};

  const std::chrono::milliseconds sdo_operation_timeout_ms_;

  // Added to the SDO timeout when waiting for a write confirmation. lely confirms or times out
  // within sdo_operation_timeout_ms_, so the margin only matters when the master is not running or
  // is starved for longer than the margin, which then also reports the write as not sent.
  static constexpr std::chrono::milliseconds kSdoConfirmationTimeoutMargin{1000};

  std::unordered_map<MotorNames, std::shared_ptr<MotorDriverInterface>> motor_drivers_;
};

class RoboteqMotorDriver : public MotorDriverInterface
{
public:
  RoboteqMotorDriver(std::weak_ptr<RoboteqDriver> driver, const std::uint8_t channel)
  : driver_(driver), channel_(channel)
  {
  }

  /**
   * @brief Reads motor state data and saves last timestamps
   */
  MotorDriverState ReadState() override;

  /**
   * @brief Sends commands to the motors
   *
   * @param cmd command value in the range [-1000, 1000]
   *
   * @exception std::runtime_error if operation fails
   */
  void SendCmdVel(const std::int32_t cmd) override;

  /**
   * @brief Sends a safety stop command to the motor connected to channel 1
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnSafetyStop() override;

private:
  std::weak_ptr<RoboteqDriver> driver_;

  const std::uint8_t channel_;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_DRIVER_HPP_
