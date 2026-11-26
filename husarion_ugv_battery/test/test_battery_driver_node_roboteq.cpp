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

#include "utils/test_battery_driver_node.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_utils/test/ros_test_utils.hpp"

class TestBatteryNodeRoboteq : public TestBatteryNode
{
public:
  TestBatteryNodeRoboteq() : TestBatteryNode(false) {}
};

TEST_F(TestBatteryNodeRoboteq, BatteryValues)
{
  // Wait for node to initialize
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(5000)));

  // Battery state msg values should be NaN
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->charge));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, battery_state_->power_supply_health);

  husarion_ugv_msgs::msg::DriverStateNamed motor_controller;
  motor_controller.state.voltage = 35.0f;
  motor_controller.state.current = 0.1f;

  RobotDriverStateMsg driver_state;
  driver_state.header.stamp = battery_driver_node_->get_clock()->now();
  driver_state.driver_states.push_back(motor_controller);
  driver_state.driver_states.push_back(motor_controller);
  driver_state_pub_->publish(driver_state);

  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(1000)));

  // This is done to check if values were read correctly, not to verify calculations.
  // If any test performing calculations fails this test will most likely fail too.
  EXPECT_FLOAT_EQ(35.0, battery_state_->voltage);
  EXPECT_FLOAT_EQ(0.2, battery_state_->current);

  // If readings stay the same values of battery_1 and battery should be the same
  EXPECT_FLOAT_EQ(battery_1_state_->voltage, battery_state_->voltage);
  EXPECT_FLOAT_EQ(battery_1_state_->current, battery_state_->current);
  EXPECT_FLOAT_EQ(battery_1_state_->percentage, battery_state_->percentage);
  EXPECT_FLOAT_EQ(battery_1_state_->charge, battery_state_->charge);
}

TEST_F(TestBatteryNodeRoboteq, BatteryTimeout)
{
  // Wait for node to initialize
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(5000)));

  // Battery state msg values should be NaN
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->charge));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, battery_state_->power_supply_health);

  // Publish some values
  husarion_ugv_msgs::msg::DriverStateNamed motor_controller;
  motor_controller.state.voltage = 35.0f;
  motor_controller.state.current = 0.1f;

  RobotDriverStateMsg driver_state;
  driver_state.header.stamp = battery_driver_node_->get_clock()->now();
  driver_state.driver_states.push_back(motor_controller);
  driver_state.driver_states.push_back(motor_controller);
  driver_state_pub_->publish(driver_state);

  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(1000)));

  // Battery state msg should have some values
  EXPECT_FALSE(std::isnan(battery_state_->voltage));
  EXPECT_FALSE(std::isnan(battery_state_->current));
  EXPECT_FALSE(std::isnan(battery_state_->charge));
  EXPECT_FALSE(std::isnan(battery_state_->percentage));

  // Wait for timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(1000)));

  // Battery state msg values should be NaN
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->charge));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, battery_state_->power_supply_health);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
