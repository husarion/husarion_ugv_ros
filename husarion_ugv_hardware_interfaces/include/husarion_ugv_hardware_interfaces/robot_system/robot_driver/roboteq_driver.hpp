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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_DRIVER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
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
    std::lock_guard<std::mutex> lck(position_timestamp_mtx_);
    return last_position_timestamps_.at(channel);
  }

  /**
   * @brief Returns the last timestamp of the speed and current data for the given channel
   */
  timespec GetSpeedCurrentTimestamp(const std::uint8_t channel)
  {
    std::lock_guard<std::mutex> lck(speed_current_timestamp_mtx_);
    return last_speed_current_timestamps_.at(channel);
  }

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

  std::mutex position_timestamp_mtx_;
  std::map<std::uint8_t, timespec> last_position_timestamps_ = {{kChannel1, {}}, {kChannel2, {}}};

  std::mutex speed_current_timestamp_mtx_;
  std::map<std::uint8_t, timespec> last_speed_current_timestamps_ = {
    {kChannel1, {}}, {kChannel2, {}}};

  std::mutex flags_current_timestamp_mtx_;
  timespec flags_current_timestamp_;

  std::mutex voltages_temps_timestamp_mtx_;
  timespec last_voltages_temps_timestamp_;

  const std::chrono::milliseconds sdo_operation_timeout_ms_;

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

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_DRIVER_HPP_
