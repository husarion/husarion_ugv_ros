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

#include "husarion_ugv_battery/battery/adc_battery.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_utils/moving_average.hpp"

namespace husarion_ugv_battery
{

ADCBattery::ADCBattery(
  const std::function<float()> & read_voltage, const std::function<float()> & read_current,
  const std::function<float()> & read_temp, const std::function<float()> & read_charge,
  const ADCBatteryParams & params)
: ReadVoltage(read_voltage), ReadCurrent(read_current), ReadTemp(read_temp), ReadCharge(read_charge)
{
  voltage_ma_ = std::make_unique<husarion_ugv_utils::MovingAverage<float>>(
    params.voltage_window_len, std::numeric_limits<float>::quiet_NaN());
  current_ma_ = std::make_unique<husarion_ugv_utils::MovingAverage<float>>(
    params.current_window_len, std::numeric_limits<float>::quiet_NaN());
  temp_ma_ = std::make_unique<husarion_ugv_utils::MovingAverage<float>>(
    params.temp_window_len, std::numeric_limits<float>::quiet_NaN());
  charge_ma_ = std::make_unique<husarion_ugv_utils::MovingAverage<float>>(
    params.charge_window_len, std::numeric_limits<float>::quiet_NaN());
}

bool ADCBattery::Present()
{
  float V_temp_sum = 0.0f;

  for (int i = 0; i < kBatPresentMeanLen; i++) {
    V_temp_sum += ReadTemp();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  const float V_temp_bat = V_temp_sum / static_cast<float>(kBatPresentMeanLen);

  return V_temp_bat < kBatDetectTresh + std::numeric_limits<float>::epsilon();
}

void ADCBattery::Update(const rclcpp::Time & header_stamp, const bool charger_connected)
{
  voltage_raw_ = ADCToBatteryVoltage(ReadVoltage());
  current_raw_ = ADCToBatteryCurrent(ReadCurrent());
  temp_raw_ = ADCToBatteryTemp(ReadTemp());
  charge_raw_ = ADCToBatteryCharge(ReadCharge());
  voltage_ma_->Roll(voltage_raw_);
  current_ma_->Roll(current_raw_);
  temp_ma_->Roll(temp_raw_);
  charge_ma_->Roll(charge_raw_);

  UpdateBatteryMsgs(header_stamp, charger_connected);
}

void ADCBattery::Reset(const rclcpp::Time & header_stamp)
{
  voltage_ma_->Reset();
  current_ma_->Reset();
  temp_ma_->Reset();
  charge_ma_->Reset();

  ResetBatteryMsgs(header_stamp);
  SetErrorMsg("");
}

inline float ADCBattery::ADCToBatteryVoltage(const float adc_data) const
{
  return adc_data * kBatVoltageFactor;
}

inline float ADCBattery::ADCToBatteryCurrent(const float adc_data) const
{
  return adc_data * kBatCurrentFactor;
}

inline float ADCBattery::ADCToBatteryCharge(const float adc_data) const
{
  return adc_data * kBatChargeFactor;
}

float ADCBattery::ADCToBatteryTemp(const float adc_data) const
{
  if (fabs(adc_data) < std::numeric_limits<float>::epsilon() || adc_data >= kUSupply) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  const float R_therm = (adc_data * kR1) / (kUSupply - adc_data);
  return (kTempCoeffA * kTempCoeffB / (kTempCoeffA * logf(R_therm / kR0) + kTempCoeffB)) -
         kKelvinToCelsiusOffset;
}

void ADCBattery::UpdateBatteryMsgs(const rclcpp::Time & header_stamp, const bool charger_connected)
{
  UpdateBatteryState(header_stamp, charger_connected);
  UpdateBatteryStateRaw();
  UpdateChargingStatus(header_stamp, charger_connected);
}

void ADCBattery::UpdateBatteryState(const rclcpp::Time & header_stamp, const bool charger_connected)
{
  const float V_bat = voltage_ma_->GetAverage();
  const float I_bat = current_ma_->GetAverage();
  const float temp_bat = temp_ma_->GetAverage();
  const float I_charge = charge_ma_->GetAverage();

  battery_state_.header.stamp = header_stamp;
  battery_state_.voltage = V_bat;
  battery_state_.temperature = temp_bat;
  battery_state_.current = -I_bat + I_charge;
  battery_state_.percentage = GetBatteryPercent(V_bat);
  battery_state_.capacity = std::numeric_limits<float>::quiet_NaN();
  battery_state_.design_capacity = kDesignedCapacity;
  battery_state_.charge = battery_state_.percentage * battery_state_.design_capacity;
  battery_state_.cell_voltage = std::vector<float>(
    kNumberOfCells, std::numeric_limits<float>::quiet_NaN());
  battery_state_.cell_temperature = std::vector<float>(
    kNumberOfCells, std::numeric_limits<float>::quiet_NaN());
  battery_state_.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_state_.present = true;
  battery_state_.location = kLocation;
  battery_state_.power_supply_status = GetBatteryStatus(charger_connected);
  battery_state_.power_supply_health = GetBatteryHealth(V_bat, temp_bat);
}

void ADCBattery::UpdateBatteryStateRaw()
{
  battery_state_raw_ = battery_state_;
  battery_state_raw_.voltage = voltage_raw_;
  battery_state_raw_.temperature = temp_raw_;
  battery_state_raw_.current = -current_raw_ + charge_raw_;
  battery_state_raw_.percentage = GetBatteryPercent(voltage_raw_);
  battery_state_raw_.charge = battery_state_raw_.percentage * battery_state_raw_.design_capacity;
}

void ADCBattery::UpdateChargingStatus(
  const rclcpp::Time & header_stamp, const bool charger_connected)
{
  charging_status_.header.stamp = header_stamp;
  charging_status_.charging = IsCharging(charger_connected);
  charging_status_.current = charge_ma_->GetAverage();
  charging_status_.charger_type = ChargingStatusMsg::UNKNOWN;  // Currently not supported
}

std::uint8_t ADCBattery::GetBatteryStatus(const bool charger_connected)
{
  if (!charger_connected) {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
  }

  if (fabs(battery_state_.percentage - 1.0f) < std::numeric_limits<float>::epsilon()) {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_FULL;
  }

  if (IsCharging(charger_connected)) {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
  }

  return BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING;
}

std::uint8_t ADCBattery::GetBatteryHealth(const float voltage, const float temp)
{
  if (voltage < kVBatFatalMin) {
    SetErrorMsg("Battery voltage is critically low!");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
  } else if (temp >= kOverheatBatTemp) {
    SetErrorMsg("Battery is overheating!");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
  } else if (voltage > kVBatFatalMax) {
    SetErrorMsg("Battery overvoltage!");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  } else if (temp < kLowBatTemp) {
    SetErrorMsg("The battery is too cold! It may result in reduced effectiveness.");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD;
  } else {
    SetErrorMsg("");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
  }
}

bool ADCBattery::IsCharging(const bool charger_connected) const
{
  if (!charger_connected) {
    return false;
  }

  return (charge_ma_->GetAverage() > kChargingCurrentTresh) ||
         (voltage_ma_->GetAverage() > kBatteryCCCheckTresh);
}

}  // namespace husarion_ugv_battery
