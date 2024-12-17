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

#ifndef HUSARION_UGV_BATTERY_BATTERY_ROBOTEQ_BATTERY_HPP_
#define HUSARION_UGV_BATTERY_BATTERY_ROBOTEQ_BATTERY_HPP_

#include <cstdint>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_msgs/msg/driver_state_named.hpp"
#include "husarion_ugv_msgs/msg/robot_driver_state.hpp"

#include "husarion_ugv_battery/battery/battery.hpp"
#include "husarion_ugv_utils/moving_average.hpp"

namespace husarion_ugv_battery
{

using RobotDriverStateMsg = husarion_ugv_msgs::msg::RobotDriverState;
using DriverStateNamedMsg = husarion_ugv_msgs::msg::DriverStateNamed;

struct RoboteqBatteryParams
{
  const float driver_state_timeout;
  const std::size_t voltage_window_len;
  const std::size_t current_window_len;
};

class RoboteqBattery : public Battery
{
public:
  RoboteqBattery(
    const std::function<RobotDriverStateMsg::SharedPtr()> & get_driver_state,
    const RoboteqBatteryParams & params);

  ~RoboteqBattery() {}

  bool Present() override;
  void Update(const rclcpp::Time & header_stamp, const bool /* charger_connected */) override;
  void Reset(const rclcpp::Time & header_stamp) override;

  float GetLoadCurrent() override { return std::numeric_limits<float>::quiet_NaN(); }

protected:
  void ValidateRobotDriverStateMsg(const rclcpp::Time & header_stamp);

private:
  void UpdateBatteryMsgs(const rclcpp::Time & header_stamp);
  void UpdateBatteryState(const rclcpp::Time & header_stamp);
  void UpdateBatteryStateRaw();
  void UpdateChargingStatus(const rclcpp::Time & header_stamp);
  std::uint8_t GetBatteryHealth(const float voltage);
  bool DriverStateHeartbeatTimeout();

  std::function<RobotDriverStateMsg::SharedPtr()> GetRobotDriverState;

  const float driver_state_timeout_;
  float voltage_raw_;
  float current_raw_;
  RobotDriverStateMsg::SharedPtr driver_state_;

  std::unique_ptr<husarion_ugv_utils::MovingAverage<float>> voltage_ma_;
  std::unique_ptr<husarion_ugv_utils::MovingAverage<float>> current_ma_;
};

}  // namespace husarion_ugv_battery

#endif  // HUSARION_UGV_BATTERY_BATTERY_ROBOTEQ_BATTERY_HPP_
