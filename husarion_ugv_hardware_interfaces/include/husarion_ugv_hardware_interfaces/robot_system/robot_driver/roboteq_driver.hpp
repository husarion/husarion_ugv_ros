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

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"

namespace husarion_ugv_hardware_interfaces
{

// Forward declaration
class RoboteqMotorDriver;

// Motor and driver state timestamps are shared between the CANopen callback thread and
// the control loop. Keeping them as a single 64-bit integer of nanoseconds makes the read
// and the write atomic on arm64, so the control loop can read them without taking a mutex
// (see the atomic members in RoboteqDriver).
inline std::int64_t TimespecToNanoseconds(const timespec & ts)
{
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL +
         static_cast<std::int64_t>(ts.tv_nsec);
}

inline timespec NanosecondsToTimespec(const std::int64_t ns)
{
  timespec ts;
  ts.tv_sec = static_cast<std::time_t>(ns / 1000000000LL);
  ts.tv_nsec = static_cast<long>(ns % 1000000000LL);
  return ts;
}

/**
 * @brief Hardware implementation of Driver with lely LoopDriver for Roboteq drivers
 * control
 */
class RoboteqDriver : public DriverInterface, public lely::canopen::LoopDriver
{
public:
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
   * @brief Blocking SDO write operation
   *
   * @exception std::runtime_error if operation fails
   */
  template <typename T>
  void SyncSDOWrite(const std::uint16_t index, const std::uint8_t subindex, const T data);

  /**
   * @brief Returns the last timestamp of the position data for the given channel
   */
  timespec GetPositionTimestamp(const std::uint8_t channel)
  {
    return NanosecondsToTimespec(
      last_position_timestamps_ns_.at(channel - kChannel1).load(std::memory_order_acquire));
  }

  /**
   * @brief Returns the last timestamp of the speed and current data for the given channel
   */
  timespec GetSpeedCurrentTimestamp(const std::uint8_t channel)
  {
    return NanosecondsToTimespec(
      last_speed_current_timestamps_ns_.at(channel - kChannel1).load(std::memory_order_acquire));
  }

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

  // Hand a velocity command to the event loop instead of writing the mapped
  // TPDO object from the caller. lely's channel I/O is only safe on the thread
  // running the loop, and every tpdo_mapped access takes the device mutex the
  // loop needs for each received frame - at 100 Hz from the control loop that
  // stole the mutex out from under RPDO dispatch and left motor state
  // timestamps ageing past the staleness deadline while the wire stayed clean.
  void PostCmdVel(const std::uint8_t channel, const std::int32_t cmd);


  // One in-flight task per channel carrying whatever the newest command is at
  // the moment it runs. A plain post per cycle would queue behind a busy loop
  // and then drive the robot through a burst of commands it has already
  // superseded - the freshest value is the only correct one for a velocity
  // setpoint.
  std::array<std::atomic<std::int32_t>, 2> pending_cmd_{};
  std::array<std::atomic<bool>, 2> cmd_task_queued_{};

  static constexpr std::uint8_t kChannel1 = 1;
  static constexpr std::uint8_t kChannel2 = 2;

private:
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

  // Written by the CANopen callback thread (OnRpdoWrite) and read by the control loop.
  // Stored as atomic nanoseconds, indexed by channel - kChannel1, so the control loop reads
  // them without taking a mutex. These used to be guarded by a std::mutex, but the control
  // loop could block on it when the CAN thread was preempted while holding it, which stalled
  // the loop long enough to time out the PDO and latch the e-stop.
  std::array<std::atomic<std::int64_t>, 2> last_position_timestamps_ns_{};
  std::array<std::atomic<std::int64_t>, 2> last_speed_current_timestamps_ns_{};
  std::atomic<std::int64_t> flags_current_timestamp_ns_{0};
  std::atomic<std::int64_t> last_voltages_temps_timestamp_ns_{0};

  // Mirrors of the mapped RPDO values, written on the CANopen thread in
  // OnRpdoWrite and read lock-free by the control loop. Reading rpdo_mapped
  // from the 100 Hz read path takes the lely device mutex once per access -
  // 13 locked accesses per cycle - and contends with the event loop mid
  // frame burst; measured as multi-ms read() spikes under camera plus drive
  // load. Same single-writer pattern as the timestamps above.
  std::array<std::atomic<std::int32_t>, 2> last_positions_{};
  std::array<std::atomic<std::int16_t>, 2> last_velocities_{};
  std::array<std::atomic<std::int16_t>, 2> last_currents_{};
  std::atomic<std::int32_t> last_flags_{0};
  std::atomic<std::int16_t> last_mcu_temp_{0};
  std::atomic<std::int16_t> last_heatsink_temp_{0};
  std::atomic<std::uint16_t> last_battery_voltage_{0};
  std::atomic<std::int16_t> last_battery_current_1_{0};
  std::atomic<std::int16_t> last_battery_current_2_{0};

  const std::chrono::milliseconds sdo_operation_timeout_ms_;

  // Local backstop added on top of the lely-side SDO timeout when waiting for
  // the write confirmation. Normally lely answers (success or timeout error)
  // within sdo_operation_timeout_ms_; the margin only matters when the master
  // is being torn down/resynced and the confirmation never arrives — the
  // caller (e.g. the e_stop_reset service callback) must never block forever.
  static constexpr std::chrono::milliseconds kSdoConfirmationTimeoutMargin{1000};

  // Consecutive confirmation-deadline misses. A single miss can be a torn-down
  // master mid-resync, but the master can also wedge with only its SDO client
  // dead - PDOs and heartbeats keep flowing, so the PDO resync watchdog never
  // sees it (bit us live: a trigger landing on an in-flight e_stop_reset left
  // lely accepting writes it never put on the wire; every reset failed until a
  // container restart). Any delivered confirmation (success or abort) clears
  // the count; kMaxConsecutiveSdoDeadlineMisses in a row exits with the same
  // code as UGVSystem::UpdateCANopenResyncWatchdog so the container restart
  // machinery heals this presentation too.
  std::atomic<unsigned> consecutive_sdo_deadline_misses_{0};
  static constexpr unsigned kMaxConsecutiveSdoDeadlineMisses = 3;
  static constexpr int kSdoWedgeExitCode = 74;  // EX_IOERR, same as the resync watchdog

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
