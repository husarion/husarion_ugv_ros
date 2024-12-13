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

#ifndef HUSARION_UGV_BATTERY_BATTERY_BATTERY_HPP_
#define HUSARION_UGV_BATTERY_BATTERY_BATTERY_HPP_

#include <algorithm>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "husarion_ugv_msgs/msg/charging_status.hpp"

namespace husarion_ugv_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using ChargingStatusMsg = husarion_ugv_msgs::msg::ChargingStatus;

class Battery
{
public:
  Battery() {}
  ~Battery() {}

  virtual bool Present() = 0;
  virtual void Update(const rclcpp::Time & header_stamp, const bool charger_connected) = 0;
  virtual void Reset(const rclcpp::Time & header_stamp) = 0;

  virtual float GetLoadCurrent() = 0;

  bool HasErrorMsg() const { return !error_msg_.empty(); }

  std::string GetErrorMsg() const { return error_msg_; }
  BatteryStateMsg GetBatteryMsg() const { return battery_state_; }
  BatteryStateMsg GetBatteryMsgRaw() const { return battery_state_raw_; }
  ChargingStatusMsg GetChargingStatus() const { return charging_status_; }

protected:
  void SetErrorMsg(const std::string & error_msg) { error_msg_ = error_msg; }

  float GetBatteryPercent(const float voltage) const
  {
    for (int i = 0; i < 5; i++) {
      if (voltage > battery_approx_ranges[i]) {
        return std::clamp(
          (battery_approx_a_values[i] * voltage + battery_approx_b_values[i]) / 100, 0.0f, 1.0f);
      }
    }
    return std::clamp(
      (battery_approx_a_values[5] * voltage + battery_approx_b_values[5]) / 100, 0.0f, 1.0f);
  }

  void ResetBatteryMsgs(const rclcpp::Time & header_stamp)
  {
    ResetBatteryState(header_stamp);
    ResetChargingStatus(header_stamp);
  }

  void ResetBatteryState(const rclcpp::Time & header_stamp)
  {
    battery_state_.header.stamp = header_stamp;
    battery_state_.voltage = std::numeric_limits<float>::quiet_NaN();
    battery_state_.temperature = std::numeric_limits<float>::quiet_NaN();
    battery_state_.current = std::numeric_limits<float>::quiet_NaN();
    battery_state_.percentage = std::numeric_limits<float>::quiet_NaN();
    battery_state_.capacity = std::numeric_limits<float>::quiet_NaN();
    battery_state_.design_capacity = kDesignedCapacity;
    battery_state_.charge = std::numeric_limits<float>::quiet_NaN();
    battery_state_.cell_voltage = std::vector<float>(
      kNumberOfCells, std::numeric_limits<float>::quiet_NaN());
    battery_state_.cell_temperature = std::vector<float>(
      kNumberOfCells, std::numeric_limits<float>::quiet_NaN());
    battery_state_.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state_.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state_.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
    battery_state_.present = true;
    battery_state_.location = kLocation;

    battery_state_raw_ = battery_state_;
  }

  void ResetChargingStatus(const rclcpp::Time & header_stamp)
  {
    charging_status_.header.stamp = header_stamp;
    charging_status_.charging = false;
    charging_status_.current = std::numeric_limits<float>::quiet_NaN();
    charging_status_.current_battery_1 = std::numeric_limits<float>::quiet_NaN();
    charging_status_.current_battery_2 = std::numeric_limits<float>::quiet_NaN();
    charging_status_.charger_type = ChargingStatusMsg::UNKNOWN;
  }

  static constexpr int kNumberOfCells = 10;
  static constexpr int kBatPresentMeanLen = 10;
  static constexpr float kChargingCurrentTresh = 0.1;
  static constexpr float kBatDetectTresh = 3.03;
  static constexpr float kVBatFatalMin = 27.0;
  static constexpr float kVBatFatalMax = 43.0;
  static constexpr float kLowBatTemp = -10.0;
  static constexpr float kOverheatBatTemp = 45.0;
  static constexpr float kDesignedCapacity = 20.0;
  static constexpr std::string_view kLocation = "user_compartment";

  static constexpr float battery_approx_ranges[5] = {41.25, 37.0, 36.0, 35.0, 33.7};
  static constexpr float battery_approx_a_values[6] = {8.665, 9.153, 19.8, 22.84, 10.538, 0.989};
  static constexpr float battery_approx_b_values[6] = {-258.73, -278.861, -672.6,
                                                       -782.04, -351.47,  -29.669};

  std::string error_msg_;
  BatteryStateMsg battery_state_;
  BatteryStateMsg battery_state_raw_;
  ChargingStatusMsg charging_status_;
};

}  // namespace husarion_ugv_battery

#endif  // HUSARION_UGV_BATTERY_BATTERY_BATTERY_HPP_
