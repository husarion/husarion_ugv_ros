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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOT_DRIVER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOT_DRIVER_HPP_

#include <string>
#include <vector>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_data_converters.hpp"

namespace husarion_ugv_hardware_interfaces
{

/**
 * @brief Abstract class for managing different robot drivers.
 */
class RobotDriverInterface
{
public:
  /**
   * @brief Initialize robot driver
   *
   * @exception std::runtime_error if boot fails
   */
  virtual void Initialize() = 0;

  /**
   * @brief Deinitialize robot driver
   */
  virtual void Deinitialize() = 0;

  /**
   * @brief Activate procedure for the driver
   *
   * @exception std::runtime_error if any procedure step fails
   */
  virtual void Activate() = 0;

  /**
   * @brief Updates current communication state with the drivers
   *
   * @exception std::runtime_error if error was detected
   */
  virtual void UpdateCommunicationState() = 0;

  /**
   * @brief Updates current motors' state (position, velocity, current).
   *
   * @exception std::runtime_error if error was detected
   */
  virtual void UpdateMotorsState() = 0;

  /**
   * @brief Updates current driver state (flags, temperatures, voltage, battery current)
   *
   * @exception std::runtime_error if error was detected
   */
  virtual void UpdateDriversState() = 0;

  /**
   * @brief Get data feedback from the driver
   *
   * @param name name of the data to get
   *
   * @return data feedback
   * @exception std::runtime_error if data with the given name does not exist
   */
  virtual const DriverData & GetData(const DriverNames name) = 0;

  /**
   * @brief Write speed commands to motors
   *
   * @param speeds vector of motor speeds in rad/s
   *
   * @exception std::runtime_error if vector has invalid size or send command fails
   */
  virtual void SendSpeedCommands(const std::vector<float> & speeds) = 0;

  /**
   * @brief Turns on E-stop
   *
   * @exception std::runtime_error if any operation returns error
   */
  virtual void TurnOnEStop() = 0;

  /**
   * @brief Turns off E-stop
   *
   * @exception std::runtime_error if any operation returns error
   */
  virtual void TurnOffEStop() = 0;

  /**
   * @brief Attempt to clear driver error flags. If driver faults still exist,
   * the error flags should remain active.
   */
  virtual void AttemptErrorFlagReset() = 0;

  /**
   * @brief Check if communication with drivers is working properly
   *
   * @return true if communication error occurred
   */
  virtual bool CommunicationError() = 0;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOT_DRIVER_HPP_
