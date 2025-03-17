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

#ifndef HUSARION_UGV_MANAGER_LIGHTS_MANAGER_NODE_HPP_
#define HUSARION_UGV_MANAGER_LIGHTS_MANAGER_NODE_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

#include "husarion_ugv_msgs/msg/led_animation.hpp"

#include "husarion_ugv_utils/moving_average.hpp"

#include "husarion_ugv_manager/behavior_tree_manager.hpp"
#include "husarion_ugv_manager/lights_manager_parameters.hpp"

namespace husarion_ugv_manager
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using BoolMsg = std_msgs::msg::Bool;
using LEDAnimationMsg = husarion_ugv_msgs::msg::LEDAnimation;
using JoyMsg = sensor_msgs::msg::Joy;

/**
 * @brief This class is responsible for creating a BehaviorTree responsible for lights management,
 * spinning it, and updating blackboard entries based on subscribed topics.
 */
class LightsManagerNode : public rclcpp::Node
{
public:
  LightsManagerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LightsManagerNode() {}

  void Initialize();

protected:
  void DeclareParameters();
  void RegisterBehaviorTree();
  std::map<std::string, std::any> CreateLightsInitialBlackboard();

  /**
   * @brief Checks whether the required blackboard entries for the lights tree are present. These
   * entries are usually updated when the first ROS message containing required information is
   * received.
   *
   * @return True if all required blackboard entries are present.
   */
  bool SystemReady();

  std::unique_ptr<BehaviorTreeManager> lights_tree_manager_;

private:
  void BatteryCB(const BatteryStateMsg::SharedPtr battery);
  void EStopCB(const BoolMsg::SharedPtr e_stop);
  void JoyCB(const JoyMsg::SharedPtr joy);
  void LightsTreeTimerCB();

  static constexpr std::size_t kDeadManButtonIndex = 4;

  float update_charging_anim_step_;

  std::shared_ptr<lights_manager::ParamListener> param_listener_;
  lights_manager::Params params_;

  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr lights_tree_timer_;

  std::unique_ptr<husarion_ugv_utils::MovingAverage<double>> battery_percent_ma_;
  BT::BehaviorTreeFactory factory_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_LIGHTS_MANAGER_NODE_HPP_
