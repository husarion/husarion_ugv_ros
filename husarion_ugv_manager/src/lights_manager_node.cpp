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

#include "husarion_ugv_manager/lights_manager_node.hpp"

#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "husarion_ugv_utils/moving_average.hpp"

#include "husarion_ugv_manager/behavior_tree_manager.hpp"
#include "husarion_ugv_manager/behavior_tree_utils.hpp"
#include "husarion_ugv_manager/lights_manager_parameters.hpp"

namespace husarion_ugv_manager
{

LightsManagerNode::LightsManagerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  this->param_listener_ =
    std::make_shared<lights_manager::ParamListener>(this->get_node_parameters_interface());
  this->params_ = this->param_listener_->get_params();

  const auto battery_percent_window_len = this->params_.battery.percent.window_len;

  battery_percent_ma_ = std::make_unique<husarion_ugv_utils::MovingAverage<double>>(
    battery_percent_window_len, 1.0);

  const auto bt_server_port = this->get_parameter("bt_server_port").as_int();
  const auto initial_blackboard = CreateLightsInitialBlackboard();
  lights_tree_manager_ = std::make_unique<BehaviorTreeManager>(
    "Lights", initial_blackboard, bt_server_port);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void LightsManagerNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  RegisterBehaviorTree();
  lights_tree_manager_->Initialize(factory_);

  using namespace std::placeholders;

  battery_sub_ = this->create_subscription<BatteryStateMsg>(
    "battery/battery_status", 10, std::bind(&LightsManagerNode::BatteryCB, this, _1));
  e_stop_sub_ = this->create_subscription<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&LightsManagerNode::EStopCB, this, _1));
  joy_sub_ = this->create_subscription<JoyMsg>(
    "joy", 10, std::bind(&LightsManagerNode::JoyCB, this, _1));

  const double timer_freq = this->params_.timer_frequency;
  const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

  lights_tree_timer_ = this->create_wall_timer(
    timer_period, std::bind(&LightsManagerNode::LightsTreeTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void LightsManagerNode::RegisterBehaviorTree()
{
  const auto bt_project_path = this->params_.bt_project_path;

  const auto plugin_libs = this->params_.plugin_libs;
  const auto ros_plugin_libs = this->params_.ros_plugin_libs;

  const auto service_availability_timeout = this->params_.ros_communication_timeout.availability;
  const auto service_response_timeout = this->params_.ros_communication_timeout.response;

  BT::RosNodeParams params;
  params.nh = this->shared_from_this();
  auto wait_for_server_timeout_s = std::chrono::duration<double>(service_availability_timeout);
  params.wait_for_server_timeout =
    std::chrono::duration_cast<std::chrono::milliseconds>(wait_for_server_timeout_s);
  auto server_timeout_s = std::chrono::duration<double>(service_response_timeout);
  params.server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(server_timeout_s);

  behavior_tree_utils::RegisterBehaviorTree(
    factory_, bt_project_path, plugin_libs, params, ros_plugin_libs);

  RCLCPP_INFO_STREAM(
    this->get_logger(), "BehaviorTree registered from path '" << bt_project_path << "'");
}

std::map<std::string, std::any> LightsManagerNode::CreateLightsInitialBlackboard()
{
  update_charging_anim_step_ = this->params_.battery.charging_anim_step;
  const float critical_battery_threshold_percent =
    static_cast<float>(this->params_.battery.percent.threshold.critical);
  const float low_battery_anim_period = static_cast<float>(this->params_.battery.anim_period.low);
  const float low_battery_threshold_percent =
    static_cast<float>(this->params_.battery.percent.threshold.low);

  const std::string undefined_charging_anim_percent = "";
  const int undefined_anim_id = -1;

  const std::map<std::string, std::any> lights_initial_bb = {
    {"charging_anim_percent", undefined_charging_anim_percent},
    {"current_anim_id", undefined_anim_id},
    {"current_battery_anim_id", undefined_anim_id},
    {"current_error_anim_id", undefined_anim_id},
    {"drive_state", false},
    {"CRITICAL_BATTERY_THRESHOLD_PERCENT", critical_battery_threshold_percent},
    {"LOW_BATTERY_ANIM_PERIOD", low_battery_anim_period},
    {"LOW_BATTERY_THRESHOLD_PERCENT", low_battery_threshold_percent},
    // anim constants
    {"E_STOP_ANIM_ID", unsigned(LEDAnimationMsg::E_STOP)},
    {"READY_ANIM_ID", unsigned(LEDAnimationMsg::READY)},
    {"ERROR_ANIM_ID", unsigned(LEDAnimationMsg::ERROR)},
    {"NO_ERROR_ANIM_ID", unsigned(LEDAnimationMsg::NO_ERROR)},
    {"MANUAL_ACTION_ANIM_ID", unsigned(LEDAnimationMsg::MANUAL_ACTION)},
    {"LOW_BATTERY_ANIM_ID", unsigned(LEDAnimationMsg::LOW_BATTERY)},
    {"CRITICAL_BATTERY_ANIM_ID", unsigned(LEDAnimationMsg::CRITICAL_BATTERY)},
    {"CHARGING_BATTERY_ANIM_ID", unsigned(LEDAnimationMsg::CHARGING_BATTERY)},
    {"BATTERY_CHARGED_ANIM_ID", unsigned(LEDAnimationMsg::BATTERY_CHARGED)},
    {"CHARGER_INSERTED_ANIM_ID", unsigned(LEDAnimationMsg::CHARGER_INSERTED)},
    {"BATTERY_NOMINAL_ANIM_ID", unsigned(LEDAnimationMsg::BATTERY_NOMINAL)},
    {"AUTONOMOUS_READY_ANIM_ID", unsigned(LEDAnimationMsg::AUTONOMOUS_READY)},
    {"AUTONOMOUS_ACTION_ANIM_ID", unsigned(LEDAnimationMsg::AUTONOMOUS_ACTION)},
    {"GOAL_ACHIEVED_ANIM_ID", unsigned(LEDAnimationMsg::GOAL_ACHIEVED)},
    {"BLINKER_LEFT_ANIM_ID", unsigned(LEDAnimationMsg::BLINKER_LEFT)},
    {"BLINKER_RIGHT_ANIM_ID", unsigned(LEDAnimationMsg::BLINKER_RIGHT)},
    // battery status constants
    {"POWER_SUPPLY_STATUS_UNKNOWN", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN)},
    {"POWER_SUPPLY_STATUS_CHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING)},
    {"POWER_SUPPLY_STATUS_DISCHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING)},
    {"POWER_SUPPLY_STATUS_NOT_CHARGING",
     unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING)},
    {"POWER_SUPPLY_STATUS_FULL", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL)},
    // battery health constants
    {"POWER_SUPPLY_HEALTH_OVERHEAT", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT)},
  };

  RCLCPP_INFO(this->get_logger(), "Blackboard created.");
  return lights_initial_bb;
}

void LightsManagerNode::BatteryCB(const BatteryStateMsg::SharedPtr battery_state)
{
  const auto battery_status = battery_state->power_supply_status;
  const auto battery_health = battery_state->power_supply_health;
  lights_tree_manager_->GetBlackboard()->set<unsigned>("battery_status", battery_status);
  lights_tree_manager_->GetBlackboard()->set<unsigned>("battery_health", battery_health);

  // don't update battery percentage if unknown status or health
  if (
    battery_status != BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN &&
    battery_health != BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
    battery_percent_ma_->Roll(battery_state->percentage);
  }

  lights_tree_manager_->GetBlackboard()->set<float>(
    "battery_percent", battery_percent_ma_->GetAverage());
  lights_tree_manager_->GetBlackboard()->set<std::string>(
    "battery_percent_round",
    std::to_string(
      round(battery_percent_ma_->GetAverage() / update_charging_anim_step_) *
      update_charging_anim_step_));
}

void LightsManagerNode::EStopCB(const BoolMsg::SharedPtr e_stop)
{
  lights_tree_manager_->GetBlackboard()->set<bool>("e_stop_state", e_stop->data);
}

void LightsManagerNode::JoyCB(const JoyMsg::SharedPtr joy)
{
  lights_tree_manager_->GetBlackboard()->set<bool>(
    "drive_state", joy->buttons[kDeadManButtonIndex]);
}

void LightsManagerNode::LightsTreeTimerCB()
{
  if (!SystemReady()) {
    return;
  }

  lights_tree_manager_->TickOnce();

  if (lights_tree_manager_->GetTreeStatus() == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Lights behavior tree returned FAILURE status");
  }
}

bool LightsManagerNode::SystemReady()
{
  if (
    !lights_tree_manager_->GetBlackboard()->getEntry("e_stop_state") ||
    !lights_tree_manager_->GetBlackboard()->getEntry("battery_status")) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for required system messages to arrive.");
    return false;
  }

  return true;
}

}  // namespace husarion_ugv_manager
