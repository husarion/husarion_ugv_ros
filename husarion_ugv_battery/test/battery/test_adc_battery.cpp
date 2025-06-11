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

#include "husarion_ugv_battery/battery/adc_battery.hpp"
#include "husarion_ugv_utils/test/test_utils.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using ChargingStatusMsg = husarion_ugv_msgs::msg::ChargingStatus;

class TestADCBattery : public testing::Test
{
public:
  TestADCBattery();
  ~TestADCBattery() {}

protected:
  void UpdateBattery(
    const float & voltage_raw, const float & current_raw, const float & temp_raw,
    const float & charge_raw, const bool & charging);

  void TestDefaultBatteryStateMsg(
    const std::uint8_t & power_supply_status, const std::uint8_t & power_supply_health);

  void TestBatteryStateMsg(
    const float & expected_voltage, const float & expected_current, const float & expected_temp,
    const float & expected_percentage, const std::uint8_t & power_supply_status,
    const std::uint8_t & power_supply_health);

  float battery_voltage_raw_;
  float battery_current_raw_;
  float battery_temp_raw_;
  float battery_charge_raw_;

  std::unique_ptr<husarion_ugv_battery::Battery> battery_;

  BatteryStateMsg battery_state_;
  ChargingStatusMsg charging_status_;
};

TestADCBattery::TestADCBattery()
{
  husarion_ugv_battery::ADCBatteryParams params = {10, 10, 10, 10};
  battery_ = std::make_unique<husarion_ugv_battery::ADCBattery>(
    [&]() { return battery_voltage_raw_; }, [&]() { return battery_current_raw_; },
    [&]() { return battery_temp_raw_; }, [&]() { return battery_charge_raw_; }, params);
}

void TestADCBattery::UpdateBattery(
  const float & voltage_raw, const float & current_raw, const float & temp_raw,
  const float & charge_raw, const bool & charging)
{
  auto stamp = rclcpp::Time(0);

  battery_voltage_raw_ = voltage_raw;
  battery_current_raw_ = current_raw;
  battery_temp_raw_ = temp_raw;
  battery_charge_raw_ = charge_raw;

  battery_->Update(stamp, charging);
  battery_state_ = battery_->GetBatteryMsg();
  charging_status_ = battery_->GetChargingStatus();
}

void TestADCBattery::TestDefaultBatteryStateMsg(
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

void TestADCBattery::TestBatteryStateMsg(
  const float & expected_voltage, const float & expected_current, const float & expected_temp,
  const float & expected_percentage, const std::uint8_t & power_supply_status,
  const std::uint8_t & power_supply_health)
{
  // Const values
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_FLOAT_EQ(expected_temp, battery_state_.temperature);
  EXPECT_TRUE(husarion_ugv_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_voltage));
  EXPECT_TRUE(
    husarion_ugv_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_temperature));
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

TEST_F(TestADCBattery, BatteryPresent)
{
  battery_temp_raw_ = 1.0;
  EXPECT_TRUE(battery_->Present());
  battery_temp_raw_ = 3.03;
  EXPECT_FALSE(battery_->Present());
  battery_temp_raw_ = 3.5;
  EXPECT_FALSE(battery_->Present());
}

TEST_F(TestADCBattery, BatteryMsgUnknown)
{
  auto stamp = rclcpp::Time(0);
  battery_->Reset(stamp);
  battery_state_ = battery_->GetBatteryMsg();

  TestDefaultBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestADCBattery, BatteryMsgValues)
{
  const auto voltage_factor = 25.04255;
  const auto current_factor = 20.0;
  const auto charge_factor = 2.5;

  const float voltage_raw_1 = 1.5;
  const float current_raw_1 = 0.01;
  const float temp_raw_1 = 1.5;
  const float charge_raw_1 = 0.5;
  UpdateBattery(voltage_raw_1, current_raw_1, temp_raw_1, charge_raw_1, false);

  float expected_voltage = voltage_raw_1 * voltage_factor;
  float expected_percentage = 0.64960694;
  float expected_temp = 28.875206;
  float expected_current = -(current_raw_1 * current_factor) + (charge_raw_1 * charge_factor);
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_temp, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // Check if values are smoothed out with moving average.
  const float voltage_raw_2 = 1.6;
  const float current_raw_2 = 0.02;
  const float temp_raw_2 = 1.4;
  const float charge_raw_2 = 0.4;
  UpdateBattery(voltage_raw_2, current_raw_2, temp_raw_2, charge_raw_2, false);

  const float voltage_mean = (voltage_raw_1 + voltage_raw_2) / 2.0;
  const float current_mean = (current_raw_1 + current_raw_2) / 2.0;
  const float charge_mean = (charge_raw_1 + charge_raw_2) / 2.0;
  expected_voltage = voltage_mean * voltage_factor;
  expected_percentage = 0.76421416;
  expected_temp = 30.306725;
  expected_current = -(current_mean * current_factor) + (charge_mean * charge_factor);
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_temp, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // Raw battery message should depend only on last readings
  battery_state_ = battery_->GetBatteryMsgRaw();
  expected_voltage = voltage_raw_2 * voltage_factor;
  expected_percentage = 0.87882143;
  expected_temp = 31.738245;
  expected_current = -(current_raw_2 * current_factor) + (charge_raw_2 * charge_factor);
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_temp, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgHealthDead)
{
  UpdateBattery(1.0, 0.01, 1.5, 0.5, false);

  EXPECT_FLOAT_EQ(0.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgHealthOvervoltage)
{
  UpdateBattery(1.72, 0.01, 1.5, 0.5, false);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgHealthOverheat)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.5, false);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgHealthCold)
{
  UpdateBattery(1.5, 0.01, 2.81, 0.5, false);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgStatusFull)
{
  UpdateBattery(1.69, 0.01, 0.98, 0.5, true);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL, battery_state_.power_supply_status);
}

TEST_F(TestADCBattery, BatteryMsgStatusCharging)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.5, true);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, battery_state_.power_supply_status);

  // Small charge, but voltage over 41.2 V
  UpdateBattery(1.65, 0.01, 0.98, 0.04, true);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, battery_state_.power_supply_status);
}

TEST_F(TestADCBattery, BatteryMsgStatusNotCharging)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.04, true);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING, battery_state_.power_supply_status);
}

TEST_F(TestADCBattery, ChargingStatusMsgCharging)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.5, true);

  EXPECT_TRUE(charging_status_.charging);
}

TEST_F(TestADCBattery, ChargingStatusMsgNotCharging)
{
  // Charger connected but not charging
  UpdateBattery(1.5, 0.01, 0.98, 0.04, true);

  EXPECT_EQ(charging_status_.charging, false);

  // Charger disconnected
  UpdateBattery(1.5, 0.01, 0.98, 0.5, false);

  EXPECT_FALSE(charging_status_.charging);
}

TEST_F(TestADCBattery, GetErrorMsg)
{
  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());

  // Send overvoltage
  UpdateBattery(1.72, 0.01, 1.5, 0.5, false);

  ASSERT_TRUE(battery_->HasErrorMsg());
  EXPECT_NE("", battery_->GetErrorMsg());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
