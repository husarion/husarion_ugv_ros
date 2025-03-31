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

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_ros_interface.hpp"

#include "utils/test_constants.hpp"

using RobotDriverStateMsg = husarion_ugv_msgs::msg::RobotDriverState;

class SystemROSInterfaceWrapper : public husarion_ugv_hardware_interfaces::SystemROSInterface
{
public:
  SystemROSInterfaceWrapper(const std::string & node_name) : SystemROSInterface(node_name) {}

  RobotDriverStateMsg & GetRobotDriverStateMsg() { return realtime_driver_state_publisher_->msg_; }
};

class TestSystemROSInterface : public ::testing::Test
{
public:
  TestSystemROSInterface()
  {
    system_ros_interface_ = std::make_unique<SystemROSInterfaceWrapper>("hardware_controller");
  }

  ~TestSystemROSInterface() { system_ros_interface_.reset(); }

protected:
  std::unique_ptr<SystemROSInterfaceWrapper> system_ros_interface_;
};

TEST(TestSystemROSInterfaceInitialization, NodeCreation)
{
  using husarion_ugv_hardware_interfaces::SystemROSInterface;

  std::vector<std::string> node_names;
  const std::string system_node_name = "hardware_controller";
  const std::string system_node_name_with_ns = "/" + system_node_name;

  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_system_node");

  std::unique_ptr<SystemROSInterface> system_ros_interface;

  system_ros_interface = std::make_unique<SystemROSInterface>(system_node_name);

  node_names = test_node->get_node_names();
  ASSERT_TRUE(
    std::find(node_names.begin(), node_names.end(), system_node_name_with_ns) != node_names.end());

  system_ros_interface.reset();
  node_names = test_node->get_node_names();
  ASSERT_FALSE(
    std::find(node_names.begin(), node_names.end(), system_node_name_with_ns) != node_names.end());

  // Check if it is possible to create a node once again (if everything was cleaned up properly)
  system_ros_interface = std::make_unique<SystemROSInterface>(system_node_name);
  node_names = test_node->get_node_names();
  ASSERT_TRUE(
    std::find(node_names.begin(), node_names.end(), system_node_name_with_ns) != node_names.end());
}

TEST_F(TestSystemROSInterface, UpdateMsgErrorFlags)
{
  husarion_ugv_hardware_interfaces::DriverData data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);

  husarion_ugv_hardware_interfaces::DriverState driver_state;
  driver_state.fault_flags = 0b00000001;
  driver_state.script_flags = 0b00000010;
  driver_state.runtime_stat_flag_channel_1 = 0b00001000;
  driver_state.runtime_stat_flag_channel_2 = 0b00000100;

  data.SetDriverState(driver_state, false);

  system_ros_interface_->UpdateMsgErrorFlags(
    husarion_ugv_hardware_interfaces::DriverNames::DEFAULT, data);

  const auto driver_state_msg = system_ros_interface_->GetRobotDriverStateMsg();

  EXPECT_TRUE(driver_state_msg.driver_states.at(0).state.fault_flag.overheat);
  EXPECT_TRUE(driver_state_msg.driver_states.at(0).state.script_flag.encoder_disconnected);
  EXPECT_TRUE(driver_state_msg.driver_states.at(0).state.channel_2_motor_runtime_error.loop_error);
  EXPECT_TRUE(
    driver_state_msg.driver_states.at(0).state.channel_1_motor_runtime_error.safety_stop_active);
}

TEST_F(TestSystemROSInterface, UpdateMsgDriversStates)
{
  husarion_ugv_hardware_interfaces::RoboteqDriverState state;

  const std::int16_t temp = 36;
  const std::int16_t heatsink_temp = 37;
  const std::uint16_t volt = 405;
  const std::int16_t battery_current_1 = 15;
  const std::int16_t battery_current_2 = 12;

  state.SetTemperature(temp);
  state.SetHeatsinkTemperature(heatsink_temp);
  state.SetVoltage(volt);
  state.SetBatteryCurrent1(battery_current_1);
  state.SetBatteryCurrent2(battery_current_2);

  system_ros_interface_->UpdateMsgDriversStates(
    husarion_ugv_hardware_interfaces::DriverNames::DEFAULT, state);

  const auto driver_state_msg = system_ros_interface_->GetRobotDriverStateMsg();

  EXPECT_FLOAT_EQ(
    static_cast<std::int16_t>(driver_state_msg.driver_states.at(0).state.temperature), temp);
  EXPECT_FLOAT_EQ(
    static_cast<std::int16_t>(driver_state_msg.driver_states.at(0).state.heatsink_temperature),
    heatsink_temp);
  EXPECT_FLOAT_EQ(
    static_cast<std::uint16_t>(driver_state_msg.driver_states.at(0).state.voltage * 10.0), volt);
  EXPECT_FLOAT_EQ(
    static_cast<std::int16_t>(driver_state_msg.driver_states.at(0).state.current * 10.0),
    (battery_current_1 + battery_current_2));
}

TEST_F(TestSystemROSInterface, UpdateMsgErrors)
{
  husarion_ugv_hardware_interfaces::CANErrors can_errors;
  can_errors.error = true;

  can_errors.write_pdo_cmds_error = true;
  can_errors.read_pdo_motor_states_error = false;
  can_errors.read_pdo_driver_state_error = false;

  husarion_ugv_hardware_interfaces::DriverCANErrors driver_can_errors;
  driver_can_errors.motor_states_data_timed_out = true;
  driver_can_errors.driver_state_data_timed_out = false;
  driver_can_errors.can_error = false;
  driver_can_errors.heartbeat_timeout = true;

  can_errors.driver_errors.emplace(
    husarion_ugv_hardware_interfaces::DriverNames::DEFAULT, driver_can_errors);

  system_ros_interface_->UpdateMsgErrors(can_errors);

  const auto driver_state_msg = system_ros_interface_->GetRobotDriverStateMsg();

  EXPECT_TRUE(driver_state_msg.error);

  EXPECT_TRUE(driver_state_msg.write_pdo_cmds_error);
  EXPECT_FALSE(driver_state_msg.read_pdo_motor_states_error);
  EXPECT_FALSE(driver_state_msg.read_pdo_driver_state_error);

  EXPECT_TRUE(driver_state_msg.driver_states.at(0).state.motor_states_data_timed_out);
  EXPECT_FALSE(driver_state_msg.driver_states.at(0).state.driver_state_data_timed_out);
  EXPECT_FALSE(driver_state_msg.driver_states.at(0).state.can_error);
  EXPECT_TRUE(driver_state_msg.driver_states.at(0).state.heartbeat_timeout);
}

TEST_F(TestSystemROSInterface, CreateDriverStateEntryInMsg)
{
  const auto driver_1_name = husarion_ugv_hardware_interfaces::DriverNames::DEFAULT;
  const auto driver_2_name = husarion_ugv_hardware_interfaces::DriverNames::FRONT;
  const auto driver_3_name = husarion_ugv_hardware_interfaces::DriverNames::REAR;

  auto & driver_state_msg = system_ros_interface_->GetRobotDriverStateMsg();

  ASSERT_EQ(driver_state_msg.driver_states.size(), 0);

  // check 3 different methods that can create a new entry in the message
  system_ros_interface_->UpdateMsgErrorFlags(
    driver_1_name, husarion_ugv_hardware_interfaces::DriverData(
                     husarion_ugv_hardware_interfaces_test::kDrivetrainSettings));
  system_ros_interface_->UpdateMsgDriversStates(driver_2_name, {});

  husarion_ugv_hardware_interfaces::CANErrors can_errors;
  can_errors.driver_errors.emplace(
    driver_3_name, husarion_ugv_hardware_interfaces::DriverCANErrors());
  system_ros_interface_->UpdateMsgErrors(can_errors);

  EXPECT_EQ(driver_state_msg.driver_states.size(), 3);
  EXPECT_EQ(
    driver_state_msg.driver_states.at(0).name,
    husarion_ugv_hardware_interfaces::DriverNamesToString(driver_1_name));
  EXPECT_EQ(
    driver_state_msg.driver_states.at(1).name,
    husarion_ugv_hardware_interfaces::DriverNamesToString(driver_2_name));
  EXPECT_EQ(
    driver_state_msg.driver_states.at(2).name,
    husarion_ugv_hardware_interfaces::DriverNamesToString(driver_3_name));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
