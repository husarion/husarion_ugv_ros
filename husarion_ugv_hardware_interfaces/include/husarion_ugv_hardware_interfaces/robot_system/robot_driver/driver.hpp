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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_DRIVER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include "lely/coapp/loop_driver.hpp"

namespace husarion_ugv_hardware_interfaces
{

enum class MotorNames {
  LEFT = 0,
  RIGHT,
};

enum class DriverNames {
  DEFAULT = 0,
  FRONT,
  REAR,
};

inline std::string MotorNamesToString(const MotorNames motor_name)
{
  switch (motor_name) {
    case MotorNames::LEFT:
      return "left";
    case MotorNames::RIGHT:
      return "right";
    default:
      return "unknown";
  }
}

inline std::string DriverNamesToString(const DriverNames driver_name)
{
  switch (driver_name) {
    case DriverNames::DEFAULT:
      return "default";
    case DriverNames::FRONT:
      return "front";
    case DriverNames::REAR:
      return "rear";
    default:
      return "unknown";
  }
}

}  // namespace husarion_ugv_hardware_interfaces

namespace std
{
template <>
struct hash<husarion_ugv_hardware_interfaces::MotorNames>
{
  std::size_t operator()(
    const husarion_ugv_hardware_interfaces::MotorNames & motor_name) const noexcept
  {
    return static_cast<std::size_t>(motor_name);
  }
};

template <>
struct hash<husarion_ugv_hardware_interfaces::DriverNames>
{
  std::size_t operator()(
    const husarion_ugv_hardware_interfaces::DriverNames & driver_name) const noexcept
  {
    return static_cast<std::size_t>(driver_name);
  }
};
}  // namespace std

namespace husarion_ugv_hardware_interfaces
{

struct MotorDriverState
{
  std::int32_t pos;
  std::int16_t vel;
  std::int16_t current;

  timespec pos_timestamp;
  timespec vel_current_timestamp;
};

struct DriverState
{
  std::uint8_t fault_flags;
  std::uint8_t script_flags;
  std::uint8_t runtime_stat_flag_channel_1;
  std::uint8_t runtime_stat_flag_channel_2;

  std::int16_t battery_current_1;
  std::int16_t battery_current_2;

  std::uint16_t battery_voltage;

  std::int16_t mcu_temp;
  std::int16_t heatsink_temp;

  timespec flags_current_timestamp;
  timespec voltages_temps_timestamp;
};

/**
 * @brief Abstract class for motor driver
 */
class MotorDriverInterface
{
public:
  /**
   * @brief Reads motors' state data returned from  (PDO 1 and 2) and saves
   * last timestamps
   */
  virtual MotorDriverState ReadState() = 0;

  /**
   * @brief Sends commands to the motors
   *
   * @param cmd command value in the range [-1000, 1000]
   *
   * @exception std::runtime_error if operation fails
   */
  virtual void SendCmdVel(const std::int32_t cmd) = 0;

  /**
   * @brief Sends a safety stop command to the motor
   *
   * @exception std::runtime_error if any operation returns error
   */
  virtual void TurnOnSafetyStop() = 0;
};

/**
 * @brief Abstract class that provides functionality for managing motor drivers
 */
class DriverInterface
{
public:
  /**
   * @brief Triggers boot operations
   *
   * @exception std::runtime_error if triggering boot fails
   */
  virtual std::future<void> Boot() = 0;

  /**
   * @brief Returns true if CAN error was detected.
   */
  virtual bool IsCANError() const = 0;

  /**
   * @brief Returns true if heartbeat timeout encountered.
   */
  virtual bool IsHeartbeatTimeout() const = 0;

  /**
   * @brief Reads driver state data returned from  (PDO 3 and 4): error flags, battery
   * voltage, battery currents (for channel 1 and 2, they are not the same as motor currents),
   * temperatures. Also saves the last timestamps
   */
  virtual DriverState ReadState() = 0;

  /**
   * @exception std::runtime_error if any operation returns error
   */
  virtual void ResetScript() = 0;

  /**
   * @exception std::runtime_error if any operation returns error
   */
  virtual void TurnOnEStop() = 0;

  /**
   * @exception std::runtime_error if any operation returns error
   */
  virtual void TurnOffEStop() = 0;

  /**
   * @brief Adds a motor driver to the driver
   */
  virtual void AddMotorDriver(
    const MotorNames name, std::shared_ptr<MotorDriverInterface> motor_driver) = 0;

  virtual std::shared_ptr<MotorDriverInterface> GetMotorDriver(const MotorNames name) = 0;

  /**
   * @brief Alias for a shared pointer to a Driver object.
   */
  using SharedPtr = std::shared_ptr<DriverInterface>;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_DRIVER_HPP_
