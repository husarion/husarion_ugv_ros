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

#ifndef HUSARION_UGV_MANAGER_SAFETY_MANAGER_NODE_HPP_
#define HUSARION_UGV_MANAGER_SAFETY_MANAGER_NODE_HPP_

#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "husarion_ugv_msgs/msg/io_state.hpp"
#include "husarion_ugv_msgs/msg/robot_driver_state.hpp"
#include "husarion_ugv_msgs/msg/system_status.hpp"

#include "husarion_ugv_utils/moving_average.hpp"

#include "husarion_ugv_manager/behavior_tree_manager.hpp"
#include "husarion_ugv_manager/safety_manager_parameters.hpp"

namespace husarion_ugv_manager
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using BoolMsg = std_msgs::msg::Bool;
using RobotDriverStateMsg = husarion_ugv_msgs::msg::RobotDriverState;
using IOStateMsg = husarion_ugv_msgs::msg::IOState;
using SystemStatusMsg = husarion_ugv_msgs::msg::SystemStatus;

/**
 * @brief This class is responsible for creating a BehaviorTrees responsible for safety and shutdown
 * management, spinning them, and updating blackboard entries based on subscribed topics.
 */
class SafetyManagerNode : public rclcpp::Node
{
public:
  SafetyManagerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SafetyManagerNode() {}

  void Initialize();

protected:
  void DeclareParameters();
  void RegisterBehaviorTree();
  std::map<std::string, std::any> CreateSafetyInitialBlackboard();

  /**
   * @brief Checks whether the required blackboard entries for the lights tree are present. These
   * entries are usually updated when the first ROS message containing required information is
   * received.
   *
   * @return True if all required blackboard entries are present.
   */
  bool SystemReady();

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BehaviorTreeManager> safety_tree_manager_;
  std::unique_ptr<BehaviorTreeManager> shutdown_tree_manager_;

private:
  void BatteryCB(const BatteryStateMsg::SharedPtr battery);
  void RobotDriverStateCB(const RobotDriverStateMsg::SharedPtr driver_state);
  void EStopCB(const BoolMsg::SharedPtr e_stop);
  void IOStateCB(const IOStateMsg::SharedPtr io_state);
  void SystemStatusCB(const SystemStatusMsg::SharedPtr system_status);
  void SafetyTreeTimerCB();

  void ShutdownRobot(const std::string & reason);

  static constexpr float kCriticalBatteryTemp = 55.0;
  static constexpr float kFatalBatteryTemp = 62.0;
  static constexpr char kShutdownLocalhostCommand[] =
    "dbus-send --system --print-reply --dest=org.freedesktop.login1 /org/freedesktop/login1 "
    "org.freedesktop.login1.Manager.PowerOff boolean:true";

  int driver_temp_window_len_;
  float update_charging_anim_step_;

  std::shared_ptr<safety_manager::ParamListener> param_listener_;
  safety_manager::Params params_;

  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<RobotDriverStateMsg>::SharedPtr driver_state_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<IOStateMsg>::SharedPtr io_state_sub_;
  rclcpp::Subscription<SystemStatusMsg>::SharedPtr system_status_sub_;
  rclcpp::TimerBase::SharedPtr safety_tree_timer_;

  std::unique_ptr<husarion_ugv_utils::MovingAverage<double>> battery_temp_ma_;
  std::unique_ptr<husarion_ugv_utils::MovingAverage<double>> cpu_temp_ma_;

  std::map<std::string, std::unique_ptr<husarion_ugv_utils::MovingAverage<double>>> driver_temp_ma_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_SAFETY_MANAGER_NODE_HPP_
