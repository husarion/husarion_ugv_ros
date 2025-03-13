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

#include "husarion_ugv_battery/battery/roboteq_battery.hpp"

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_utils/moving_average.hpp"

namespace husarion_ugv_battery
{

RoboteqBattery::RoboteqBattery(
  const std::function<RobotDriverStateMsg::SharedPtr()> & get_driver_state,
  const RoboteqBatteryParams & params)
: GetRobotDriverState(get_driver_state), driver_state_timeout_(params.driver_state_timeout)
{
  voltage_ma_ = std::make_unique<husarion_ugv_utils::MovingAverage<float>>(
    params.voltage_window_len, std::numeric_limits<float>::quiet_NaN());
  current_ma_ = std::make_unique<husarion_ugv_utils::MovingAverage<float>>(
    params.current_window_len, std::numeric_limits<float>::quiet_NaN());
}

bool RoboteqBattery::Present() { return true; }

void RoboteqBattery::Update(const rclcpp::Time & header_stamp, const bool /* charger_connected */)
{
  driver_state_ = GetRobotDriverState();
  ValidateRobotDriverStateMsg(header_stamp);

  float voltage = 0.0f;
  float current = 0.0f;
  std::for_each(
    driver_state_->driver_states.begin(), driver_state_->driver_states.end(),
    [&voltage, &current](const DriverStateNamedMsg & driver) {
      voltage += driver.state.voltage;
      current += driver.state.current;
    });

  voltage_raw_ = voltage / driver_state_->driver_states.size();
  current_raw_ = current;
  voltage_ma_->Roll(voltage_raw_);
  current_ma_->Roll(current_raw_);

  UpdateBatteryMsgs(header_stamp);
}

void RoboteqBattery::Reset(const rclcpp::Time & header_stamp)
{
  voltage_ma_->Reset();
  current_ma_->Reset();

  ResetBatteryMsgs(header_stamp);
  SetErrorMsg("");
}

void RoboteqBattery::ValidateRobotDriverStateMsg(const rclcpp::Time & header_stamp)
{
  if (!driver_state_) {
    throw std::runtime_error("Waiting for driver state message to arrive.");
  }

  rclcpp::Time msg_time(driver_state_->header.stamp);
  if ((header_stamp - msg_time) > rclcpp::Duration::from_seconds(driver_state_timeout_)) {
    throw std::runtime_error("Driver state message timeout.");
  }

  if (DriverStateHeartbeatTimeout()) {
    throw std::runtime_error("Motor controller heartbeat timeout error.");
  }
}

void RoboteqBattery::UpdateBatteryMsgs(const rclcpp::Time & header_stamp)
{
  UpdateBatteryState(header_stamp);
  UpdateBatteryStateRaw();
  UpdateChargingStatus(header_stamp);
}

void RoboteqBattery::UpdateBatteryState(const rclcpp::Time & header_stamp)
{
  const float V_bat = voltage_ma_->GetAverage();
  const float I_bat = current_ma_->GetAverage();

  battery_state_.header.stamp = header_stamp;
  battery_state_.voltage = V_bat;
  battery_state_.temperature = std::numeric_limits<float>::quiet_NaN();
  battery_state_.current = I_bat;
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
  battery_state_.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
  battery_state_.power_supply_health = GetBatteryHealth(V_bat);
}

void RoboteqBattery::UpdateBatteryStateRaw()
{
  battery_state_raw_ = battery_state_;
  battery_state_raw_.voltage = voltage_raw_;
  battery_state_raw_.current = current_raw_;
  battery_state_raw_.percentage = GetBatteryPercent(voltage_raw_);
  battery_state_raw_.charge = battery_state_raw_.percentage * battery_state_raw_.design_capacity;
}

void RoboteqBattery::UpdateChargingStatus(const rclcpp::Time & header_stamp)
{
  charging_status_.header.stamp = header_stamp;
  charging_status_.charging = false;
  charging_status_.current = std::numeric_limits<float>::quiet_NaN();
  charging_status_.charger_type = ChargingStatusMsg::UNKNOWN;
}

std::uint8_t RoboteqBattery::GetBatteryHealth(const float voltage)
{
  if (voltage < kVBatFatalMin) {
    SetErrorMsg("Battery voltage is critically low!");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
  } else if (voltage > kVBatFatalMax) {
    SetErrorMsg("Battery overvoltage!");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  } else {
    SetErrorMsg("");
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
  }
}

bool RoboteqBattery::DriverStateHeartbeatTimeout()
{
  return std::any_of(
    driver_state_->driver_states.begin(), driver_state_->driver_states.end(),
    [](const DriverStateNamedMsg & driver) { return driver.state.heartbeat_timeout; });
}

}  // namespace husarion_ugv_battery
