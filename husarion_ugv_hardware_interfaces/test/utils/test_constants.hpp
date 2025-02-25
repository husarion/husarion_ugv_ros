// Copyright 2023 Husarion sp. z o.o.
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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_TEST_CONSTANTS_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_TEST_CONSTANTS_HPP_

#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp>
#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp>
#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_data_converters.hpp>

namespace husarion_ugv_hardware_interfaces_test
{

const husarion_ugv_hardware_interfaces::CANopenSettings kCANopenSettings{
  "robot_can",
  3,
  {{husarion_ugv_hardware_interfaces::DriverNames::DEFAULT, 1},
   {husarion_ugv_hardware_interfaces::DriverNames::FRONT, 1},
   {husarion_ugv_hardware_interfaces::DriverNames::REAR, 2}},
  std::chrono::milliseconds(15),
  std::chrono::milliseconds(75),
  std::chrono::milliseconds(100),
};

const husarion_ugv_hardware_interfaces::DrivetrainSettings kDrivetrainSettings{
  0.11, 30.08, 0.75, 1600.0, 3600.0};

constexpr float kRadPerSecToRbtqCmd = 30.08 * (1.0 / (2.0 * M_PI)) * 60.0 * (1000.0 / 3600.0);
constexpr float kRbtqPosFbToRad = (1. / 1600) * (1.0 / 30.08) * (2.0 * M_PI);
constexpr float kRbtqVelFbToRadPerSec = (3600. / 1000.) * (1. / 30.08) * (1. / 60.) * (2.0 * M_PI);
constexpr float kRbtqCurrentFbToNewtonMeters = (1. / 10.) * 0.11 * 30.08 * 0.75;

const std::string kPantherSystemName = "wheels";

const std::string kUrdfHeader = R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
<ros2_control name="wheels" type="system">
)";

const std::string kUrdfFooter = R"(</ros2_control>
</robot>
)";

const std::string kJointInterfaces =
  R"(<command_interface name="velocity" />
<state_interface name="position" />
<state_interface name="velocity" />
<state_interface name="effort" />
)";

const std::string kPluginName =
  R"(<plugin>husarion_ugv_hardware_interfaces/PantherSystem</plugin>
)";

const std::map<std::string, std::string> kDefaultParamMap = {
  {"encoder_resolution", "1600"},
  {"gear_ratio", "30.08"},
  {"gearbox_efficiency", "0.75"},
  {"motor_torque_constant", "0.11"},
  {"max_rpm_motor_speed", "3600.0"},
  {"can_interface_name", "robot_can"},
  {"master_can_id", "3"},
  {"front_driver_can_id", "1"},
  {"rear_driver_can_id", "2"},
  {"sdo_operation_timeout_ms", "100"},
  {"pdo_motor_states_timeout_ms", "15"},
  {"pdo_driver_state_timeout_ms", "75"},
  {"driver_states_update_frequency", "20.0"},
  {"max_roboteq_initialization_attempts", "5"},
  {"max_roboteq_activation_attempts", "5"},
  {"max_write_pdo_cmds_errors_count", "4"},
  {"max_read_pdo_motor_states_errors_count", "4"},
  {"max_read_pdo_driver_state_errors_count", "20"},
};

const std::vector<std::string> kDefaultJoints = {
  "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"};

const std::string kRobotDriverStateTopic = "/hardware_controller/robot_driver_state";

}  // namespace husarion_ugv_hardware_interfaces_test

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_TEST_CONSTANTS_HPP_
