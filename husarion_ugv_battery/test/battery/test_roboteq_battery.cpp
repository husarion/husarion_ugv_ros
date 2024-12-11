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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "husarion_ugv_msgs/msg/driver_state_named.hpp"
#include "husarion_ugv_msgs/msg/robot_driver_state.hpp"

#include "husarion_ugv_battery/battery/roboteq_battery.hpp"
#include "husarion_ugv_utils/test/test_utils.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using RobotDriverStateMsg = husarion_ugv_msgs::msg::RobotDriverState;

class RoboteqBatteryWrapper : public husarion_ugv_battery::RoboteqBattery
{
public:
  RoboteqBatteryWrapper(
    const std::function<RobotDriverStateMsg::SharedPtr()> & get_driver_state,
    const husarion_ugv_battery::RoboteqBatteryParams & params)
  : RoboteqBattery(get_driver_state, params)
  {
  }

  void ValidateRobotDriverStateMsg(const rclcpp::Time & header_stamp)
  {
    return RoboteqBattery::ValidateRobotDriverStateMsg(header_stamp);
  }
};

class TestRoboteqBattery : public testing::Test
{
public:
  TestRoboteqBattery();
  ~TestRoboteqBattery() {}

protected:
  void UpdateBattery(const float voltage, const float current);
  void TestDefaultBatteryStateMsg(
    const std::uint8_t power_supply_status, const std::uint8_t power_supply_health);

  void TestBatteryStateMsg(
    const float expected_voltage, const float expected_current, const float expected_percentage,
    const std::uint8_t power_supply_status, const std::uint8_t power_supply_health);

  static constexpr float kRobotDriverStateTimeout = 0.2;

  std::unique_ptr<RoboteqBatteryWrapper> battery_;
  BatteryStateMsg battery_state_;
  RobotDriverStateMsg::SharedPtr driver_state_;
};

TestRoboteqBattery::TestRoboteqBattery()
{
  const husarion_ugv_battery::RoboteqBatteryParams params = {kRobotDriverStateTimeout, 10, 10};
  battery_ = std::make_unique<RoboteqBatteryWrapper>([&]() { return driver_state_; }, params);
}

void TestRoboteqBattery::UpdateBattery(const float voltage, const float current)
{
  if (!driver_state_) {
    driver_state_ = std::make_shared<RobotDriverStateMsg>();
    driver_state_->driver_states.push_back(husarion_ugv_msgs::msg::DriverStateNamed());
    driver_state_->driver_states.push_back(husarion_ugv_msgs::msg::DriverStateNamed());
  }

  auto stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

  driver_state_->header.stamp = stamp;
  driver_state_->driver_states.at(0).state.voltage = voltage;
  driver_state_->driver_states.at(1).state.voltage = voltage;
  driver_state_->driver_states.at(0).state.current = current;
  driver_state_->driver_states.at(1).state.current = current;

  battery_->Update(stamp, false);
  battery_state_ = battery_->GetBatteryMsg();
}

void TestRoboteqBattery::TestDefaultBatteryStateMsg(
  const std::uint8_t power_supply_status, const std::uint8_t power_supply_health)
{
  // Const values
  EXPECT_TRUE(std::isnan(battery_state_.temperature));
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_.power_supply_technology);
  EXPECT_TRUE(husarion_ugv_utils::test_utils::CheckNaNVector(battery_state_.cell_voltage));
  EXPECT_TRUE(husarion_ugv_utils::test_utils::CheckNaNVector(battery_state_.cell_temperature));
  EXPECT_TRUE(battery_state_.present);
  EXPECT_EQ("user_compartment", battery_state_.location);

  // Variable values
  EXPECT_TRUE(std::isnan(battery_state_.voltage));
  EXPECT_TRUE(std::isnan(battery_state_.current));
  EXPECT_TRUE(std::isnan(battery_state_.percentage));
  EXPECT_TRUE(std::isnan(battery_state_.charge));

  EXPECT_EQ(power_supply_status, battery_state_.power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_.power_supply_health);
}

void TestRoboteqBattery::TestBatteryStateMsg(
  const float expected_voltage, const float expected_current, const float expected_percentage,
  const std::uint8_t power_supply_status, const std::uint8_t power_supply_health)
{
  // Const values
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_TRUE(std::isnan(battery_state_.temperature));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_TRUE(husarion_ugv_utils::test_utils::CheckNaNVector(battery_state_.cell_voltage));
  EXPECT_TRUE(husarion_ugv_utils::test_utils::CheckNaNVector(battery_state_.cell_temperature));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_.power_supply_technology);
  EXPECT_EQ("user_compartment", battery_state_.location);

  // Variable values
  EXPECT_FLOAT_EQ(expected_voltage, battery_state_.voltage);
  EXPECT_FLOAT_EQ(expected_current, battery_state_.current);
  EXPECT_FLOAT_EQ(expected_percentage, battery_state_.percentage);
  EXPECT_FLOAT_EQ(expected_percentage * 20.0f, battery_state_.charge);

  EXPECT_TRUE(battery_state_.present);
  EXPECT_EQ(power_supply_status, battery_state_.power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_.power_supply_health);
}

TEST_F(TestRoboteqBattery, BatteryPresent) { EXPECT_TRUE(battery_->Present()); }

TEST_F(TestRoboteqBattery, BatteryMsgUnknown)
{
  auto stamp = rclcpp::Time(0);
  battery_->Reset(stamp);
  battery_state_ = battery_->GetBatteryMsg();

  TestDefaultBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestRoboteqBattery, BatteryMsgValues)
{
  const float voltage_1 = 35.0;
  const float current_1 = 0.1;
  UpdateBattery(voltage_1, current_1);

  float expected_voltage = voltage_1;
  float expected_percentage = 0.17360015;
  float expected_current = current_1 * 2.0;
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  const float voltage_2 = 37.0;
  const float current_2 = 0.2;
  UpdateBattery(voltage_2, current_2);

  expected_voltage = (voltage_1 + voltage_2) / 2.0;
  expected_percentage = 0.40200013;
  expected_current = (current_1 * 2.0 + current_2 * 2.0) / 2.0;
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // Check raw battery msg
  battery_state_ = battery_->GetBatteryMsgRaw();
  expected_voltage = voltage_2;
  expected_percentage = 0.60000002;
  expected_current = current_2 * 2.0;
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());
}

TEST_F(TestRoboteqBattery, BatteryMsgHealthDead)
{
  UpdateBattery(26.0, 0.1);

  EXPECT_FLOAT_EQ(0.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestRoboteqBattery, BatteryMsgHealthOvervoltage)
{
  UpdateBattery(44.0, 0.1);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestRoboteqBattery, GetErrorMsg)
{
  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());

  // Send overvoltage
  UpdateBattery(44.0, 0.1);

  ASSERT_TRUE(battery_->HasErrorMsg());
  EXPECT_NE("", battery_->GetErrorMsg());
}

TEST_F(TestRoboteqBattery, ValidateRobotDriverStateMsg)
{
  auto stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Check empty driver_state msg
  EXPECT_THROW(battery_->ValidateRobotDriverStateMsg(stamp), std::runtime_error);

  UpdateBattery(35.0f, 0.1f);
  EXPECT_NO_THROW(battery_->ValidateRobotDriverStateMsg(stamp));

  // Check timeout
  const std::uint32_t timeout_nanoseconds = (kRobotDriverStateTimeout + 0.05) * 1e9;
  stamp = rclcpp::Time(0, timeout_nanoseconds, RCL_ROS_TIME);
  EXPECT_THROW(battery_->ValidateRobotDriverStateMsg(stamp), std::runtime_error);

  // Reset error
  stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  EXPECT_NO_THROW(battery_->ValidateRobotDriverStateMsg(stamp));

  // Check heartbeat timeout error throw
  driver_state_->driver_states.at(0).state.heartbeat_timeout = true;
  EXPECT_THROW(battery_->ValidateRobotDriverStateMsg(stamp), std::runtime_error);
  driver_state_->driver_states.at(0).state.heartbeat_timeout = false;
  driver_state_->driver_states.at(1).state.heartbeat_timeout = true;
  EXPECT_THROW(battery_->ValidateRobotDriverStateMsg(stamp), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  return run_tests;
}
