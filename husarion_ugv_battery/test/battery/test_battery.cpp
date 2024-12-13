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
#include <cstdint>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "husarion_ugv_msgs/msg/charging_status.hpp"

#include "husarion_ugv_battery/battery/battery.hpp"
#include "husarion_ugv_utils/test/test_utils.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using ChargingStatusMsg = husarion_ugv_msgs::msg::ChargingStatus;

class BatteryWrapper : public husarion_ugv_battery::Battery
{
public:
  BatteryWrapper() {}

  float GetBatteryPercent(const float voltage) const { return Battery::GetBatteryPercent(voltage); }

  void ResetBatteryState(const rclcpp::Time & header_stamp)
  {
    return Battery::ResetBatteryState(header_stamp);
  }

  void ResetChargingStatus(const rclcpp::Time & header_stamp)
  {
    return Battery::ResetChargingStatus(header_stamp);
  }

  void SetErrorMsg(const std::string error_msg) { return Battery::SetErrorMsg(error_msg); }

  // Mock up methods to make wrapper a valid subclass
  bool Present() { return true; }
  void Update(const rclcpp::Time & /* header_stamp */, bool /* charger_connected */) {}
  void Reset(const rclcpp::Time & /* header_stamp */) {}
  float GetLoadCurrent() { return 0.0; }
};

class TestBattery : public testing::Test
{
public:
  TestBattery();
  ~TestBattery() {}

protected:
  void TestDefaultBatteryStateMsg(
    const std::uint8_t & power_supply_status, const std::uint8_t & power_supply_health);

  std::unique_ptr<BatteryWrapper> battery_;
  BatteryStateMsg battery_state_;
};

TestBattery::TestBattery() { battery_ = std::make_unique<BatteryWrapper>(); }

void TestBattery::TestDefaultBatteryStateMsg(
  const std::uint8_t & power_supply_status, const std::uint8_t & power_supply_health)
{
  // Const values
  EXPECT_TRUE(std::isnan(battery_state_.temperature));
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_.power_supply_technology);
  EXPECT_TRUE(husarion_ugv_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_voltage));
  EXPECT_TRUE(
    husarion_ugv_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_temperature));
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

TEST_F(TestBattery, GetErrorMsg)
{
  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());

  battery_->SetErrorMsg("error");

  ASSERT_TRUE(battery_->HasErrorMsg());
  EXPECT_EQ("error", battery_->GetErrorMsg());
}

TEST_F(TestBattery, GetBatteryPercent)
{
  EXPECT_NEAR(0.0, battery_->GetBatteryPercent(30.0f), 1e-5);
  EXPECT_NEAR(0.01979, battery_->GetBatteryPercent(32.0f), 1e-6);
  EXPECT_NEAR(0.06822, battery_->GetBatteryPercent(34.0f), 1e-6);
  EXPECT_NEAR(0.2878, battery_->GetBatteryPercent(35.5f), 1e-6);
  EXPECT_NEAR(0.501, battery_->GetBatteryPercent(36.5f), 1e-6);
  EXPECT_NEAR(0.68953, battery_->GetBatteryPercent(38.0f), 1e-6);
  EXPECT_FLOAT_EQ(1.0, battery_->GetBatteryPercent(41.4f));
  EXPECT_FLOAT_EQ(1.0, battery_->GetBatteryPercent(45.0f));
}

TEST_F(TestBattery, ResetBatteryState)
{
  auto empty_battery_msg = BatteryStateMsg();
  ASSERT_EQ(empty_battery_msg, battery_->GetBatteryMsg());

  auto stamp = rclcpp::Time(0);
  battery_->ResetBatteryState(stamp);

  battery_state_ = battery_->GetBatteryMsg();
  TestDefaultBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestBattery, ResetBatteryStateRaw)
{
  auto empty_battery_msg = BatteryStateMsg();
  ASSERT_EQ(empty_battery_msg, battery_->GetBatteryMsgRaw());

  auto stamp = rclcpp::Time(0);
  battery_->ResetBatteryState(stamp);

  battery_state_ = battery_->GetBatteryMsgRaw();
  TestDefaultBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestBattery, ResetChargingStatus)
{
  auto empty_charging_status = ChargingStatusMsg();
  ASSERT_EQ(empty_charging_status, battery_->GetChargingStatus());

  auto stamp = rclcpp::Time(0);
  battery_->ResetChargingStatus(stamp);

  const auto charging_status = battery_->GetChargingStatus();
  EXPECT_EQ(false, charging_status.charging);
  EXPECT_TRUE(std::isnan(charging_status.current));
  EXPECT_TRUE(std::isnan(charging_status.current_battery_1));
  EXPECT_TRUE(std::isnan(charging_status.current_battery_2));
  EXPECT_EQ(ChargingStatusMsg::UNKNOWN, charging_status.charger_type);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
