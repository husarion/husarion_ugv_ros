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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_LYNX_ROBOT_DRIVER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_LYNX_ROBOT_DRIVER_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_data_converters.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_robot_driver.hpp"

namespace husarion_ugv_hardware_interfaces
{

/**
 * @brief This class implements RoboteqRobotDriver for Lynx robot.
 * It defines one Roboteq controller with two motors controlling left and right wheels.
 */
class LynxRobotDriver : public RoboteqRobotDriver
{
public:
  LynxRobotDriver(
    const CANopenSettings & canopen_settings, const DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time = std::chrono::milliseconds(1000))
  : RoboteqRobotDriver(canopen_settings, drivetrain_settings, activate_wait_time)
  {
  }

  ~LynxRobotDriver() = default;

  /**
   * @brief Write speed commands to motors
   *
   * @param speeds vector of motor speeds in rad/s. The order is: left, right
   *
   * @exception std::runtime_error if invalid vector size, send command fails or CAN error is
   * detected
   */
  void SendSpeedCommands(const std::vector<float> & speeds) override;

  /**
   * @brief Attempt to clear driver error flags by sending 0 velocity commands to motors. If Roboteq
   * driver faults still exist, the error flag will remain active.
   */
  void AttemptErrorFlagReset() override { SendSpeedCommands({0.0, 0.0}); };

protected:
  /**
   * @brief This method defines driver objects and adds motor drivers for them. It is virtual to
   * allow mocking drivers in tests.
   */
  void DefineDrivers() override;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_LYNX_ROBOT_DRIVER_HPP_
