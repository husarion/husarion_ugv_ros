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

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/lynx_robot_driver.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_robot_driver.hpp"

namespace husarion_ugv_hardware_interfaces
{

void LynxRobotDriver::SendSpeedCommands(const std::vector<float> & speeds)
{
  if (speeds.size() != 2) {
    throw std::runtime_error(
      "Invalid speeds vector size. Expected 2, got " + std::to_string(speeds.size()));
  }

  const auto speed_left = this->GetCmdVelConverter().Convert(speeds.at(0));
  const auto speed_right = this->GetCmdVelConverter().Convert(speeds.at(1));

  try {
    drivers_.at(DriverNames::DEFAULT)->GetMotorDriver(MotorNames::LEFT)->SendCmdVel(speed_left);
    drivers_.at(DriverNames::DEFAULT)->GetMotorDriver(MotorNames::RIGHT)->SendCmdVel(speed_right);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Driver send Roboteq cmd failed: " + std::string(e.what()));
  }

  if (drivers_.at(DriverNames::DEFAULT)->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the Driver when trying to write speed commands.");
  }
}

void LynxRobotDriver::DefineDrivers()
{
  auto driver = std::make_shared<RoboteqDriver>(
    canopen_manager_.GetMaster(), canopen_settings_.driver_can_ids.at(DriverNames::DEFAULT),
    canopen_settings_.sdo_operation_timeout_ms);

  auto left_motor_driver = std::make_shared<RoboteqMotorDriver>(
    std::dynamic_pointer_cast<RoboteqDriver>(driver), MotorChannels::LEFT);
  auto right_motor_driver = std::make_shared<RoboteqMotorDriver>(
    std::dynamic_pointer_cast<RoboteqDriver>(driver), MotorChannels::RIGHT);

  driver->AddMotorDriver(MotorNames::LEFT, left_motor_driver);
  driver->AddMotorDriver(MotorNames::RIGHT, right_motor_driver);

  drivers_.emplace(DriverNames::DEFAULT, driver);
}

}  // namespace husarion_ugv_hardware_interfaces
