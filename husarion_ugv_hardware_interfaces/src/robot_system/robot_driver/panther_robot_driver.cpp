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

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/panther_robot_driver.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_robot_driver.hpp"

namespace husarion_ugv_hardware_interfaces
{

void PantherRobotDriver::SendSpeedCommands(const std::vector<float> & speeds)
{
  if (speeds.size() != 4) {
    throw std::runtime_error(
      "Invalid speeds vector size. Expected 4, got " + std::to_string(speeds.size()));
  }

  const auto speed_fl = this->GetCmdVelConverter().Convert(speeds.at(0));
  const auto speed_fr = this->GetCmdVelConverter().Convert(speeds.at(1));
  const auto speed_rl = this->GetCmdVelConverter().Convert(speeds.at(2));
  const auto speed_rr = this->GetCmdVelConverter().Convert(speeds.at(3));

  try {
    drivers_.at(DriverNames::FRONT)->GetMotorDriver(MotorNames::LEFT)->SendCmdVel(speed_fl);
    drivers_.at(DriverNames::FRONT)->GetMotorDriver(MotorNames::RIGHT)->SendCmdVel(speed_fr);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send Roboteq cmd failed: " + std::string(e.what()));
  }
  try {
    drivers_.at(DriverNames::REAR)->GetMotorDriver(MotorNames::LEFT)->SendCmdVel(speed_rl);
    drivers_.at(DriverNames::REAR)->GetMotorDriver(MotorNames::RIGHT)->SendCmdVel(speed_rr);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Rear driver send Roboteq cmd failed: " + std::string(e.what()));
  }

  if (drivers_.at(DriverNames::FRONT)->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the front driver when trying to write speed commands.");
  }
  if (drivers_.at(DriverNames::REAR)->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the rear driver when trying to write speed commands.");
  }
}

void PantherRobotDriver::DefineDrivers()
{
  auto front_driver = std::make_shared<RoboteqDriver>(
    canopen_manager_.GetMaster(), canopen_settings_.driver_can_ids.at(DriverNames::FRONT),
    canopen_settings_.sdo_operation_timeout_ms);
  auto rear_driver = std::make_shared<RoboteqDriver>(
    canopen_manager_.GetMaster(), canopen_settings_.driver_can_ids.at(DriverNames::REAR),
    canopen_settings_.sdo_operation_timeout_ms);

  auto fl_motor_driver = std::make_shared<RoboteqMotorDriver>(
    std::dynamic_pointer_cast<RoboteqDriver>(front_driver), MotorChannels::LEFT);
  auto fr_motor_driver = std::make_shared<RoboteqMotorDriver>(
    std::dynamic_pointer_cast<RoboteqDriver>(front_driver), MotorChannels::RIGHT);
  auto rl_motor_driver = std::make_shared<RoboteqMotorDriver>(
    std::dynamic_pointer_cast<RoboteqDriver>(rear_driver), MotorChannels::LEFT);
  auto rr_motor_driver = std::make_shared<RoboteqMotorDriver>(
    std::dynamic_pointer_cast<RoboteqDriver>(rear_driver), MotorChannels::RIGHT);

  front_driver->AddMotorDriver(MotorNames::LEFT, fl_motor_driver);
  front_driver->AddMotorDriver(MotorNames::RIGHT, fr_motor_driver);
  rear_driver->AddMotorDriver(MotorNames::LEFT, rl_motor_driver);
  rear_driver->AddMotorDriver(MotorNames::RIGHT, rr_motor_driver);

  drivers_.emplace(DriverNames::FRONT, front_driver);
  drivers_.emplace(DriverNames::REAR, rear_driver);
}

}  // namespace husarion_ugv_hardware_interfaces
