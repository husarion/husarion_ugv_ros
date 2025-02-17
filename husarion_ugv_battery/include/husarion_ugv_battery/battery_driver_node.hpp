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

#ifndef HUSARION_UGV_BATTERY_BATTERY_DRIVER_NODE_HPP_
#define HUSARION_UGV_BATTERY_BATTERY_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_msgs/msg/robot_driver_state.hpp"

#include "husarion_ugv_battery/adc_data_reader.hpp"
#include "husarion_ugv_battery/battery/battery.hpp"
#include "husarion_ugv_battery/battery_parameters.hpp"
#include "husarion_ugv_battery/battery_publisher/battery_publisher.hpp"

namespace husarion_ugv_battery
{

using RobotDriverStateMsg = husarion_ugv_msgs::msg::RobotDriverState;

class BatteryDriverNode : public rclcpp::Node
{
public:
  BatteryDriverNode(
    const std::string & node_name, const std::string & ns = "/",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void BatteryPubTimerCB();
  void Initialize();
  void InitializeWithADCBattery();
  void InitializeWithRoboteqBattery();

  static constexpr int kADCCurrentOffset = 625;

  RobotDriverStateMsg::SharedPtr driver_state_;

  std::shared_ptr<ADCDataReader> adc0_reader_;
  std::shared_ptr<ADCDataReader> adc1_reader_;
  std::shared_ptr<Battery> battery_1_;
  std::shared_ptr<Battery> battery_2_;
  std::shared_ptr<BatteryPublisher> battery_publisher_;

  std::shared_ptr<battery::ParamListener> param_listener_;
  battery::Params params_;

  rclcpp::Subscription<RobotDriverStateMsg>::SharedPtr driver_state_sub_;
  rclcpp::TimerBase::SharedPtr battery_pub_timer_;

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace husarion_ugv_battery

#endif  // HUSARION_UGV_BATTERY_BATTERY_DRIVER_NODE_HPP_
