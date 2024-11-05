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

#include "panther_manager/safety_manager_node.hpp"

#include <algorithm>
#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panther_utils/moving_average.hpp"

#include <panther_manager/behavior_tree_manager.hpp>
#include <panther_manager/behavior_tree_utils.hpp>

namespace panther_manager
{

SafetyManagerNode::SafetyManagerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  DeclareParameters();

  const auto battery_temp_window_len = this->get_parameter("battery.temp.window_len").as_int();
  const auto cpu_temp_window_len = this->get_parameter("cpu.temp.window_len").as_int();
  const auto driver_temp_window_len = this->get_parameter("driver.temp.window_len").as_int();

  battery_temp_ma_ =
    std::make_unique<panther_utils::MovingAverage<double>>(battery_temp_window_len);
  cpu_temp_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(cpu_temp_window_len);
  front_driver_temp_ma_ =
    std::make_unique<panther_utils::MovingAverage<double>>(driver_temp_window_len);
  rear_driver_temp_ma_ =
    std::make_unique<panther_utils::MovingAverage<double>>(driver_temp_window_len);

  const auto safety_initial_blackboard = CreateSafetyInitialBlackboard();
  safety_tree_manager_ = std::make_unique<BehaviorTreeManager>(
    "Safety", safety_initial_blackboard, 6666);

  const auto shutdown_hosts_path = this->get_parameter("shutdown_hosts_path").as_string();
  const std::map<std::string, std::any> shutdown_initial_blackboard = {
    {"SHUTDOWN_HOSTS_FILE", shutdown_hosts_path.c_str()},
  };
  shutdown_tree_manager_ = std::make_unique<BehaviorTreeManager>(
    "Shutdown", shutdown_initial_blackboard, 7777);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void SafetyManagerNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  RegisterBehaviorTree();
  safety_tree_manager_->Initialize(factory_);
  shutdown_tree_manager_->Initialize(factory_);

  // -------------------------------
  //   Subscribers
  // -------------------------------

  using namespace std::placeholders;

  battery_sub_ = this->create_subscription<BatteryStateMsg>(
    "battery/battery_status", 10, std::bind(&SafetyManagerNode::BatteryCB, this, _1));
  driver_state_sub_ = this->create_subscription<DriverStateMsg>(
    "hardware/motor_controllers_state", 10, std::bind(&SafetyManagerNode::DriverStateCB, this, _1));
  e_stop_sub_ = this->create_subscription<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SafetyManagerNode::EStopCB, this, _1));
  io_state_sub_ = this->create_subscription<IOStateMsg>(
    "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SafetyManagerNode::IOStateCB, this, _1));
  system_status_sub_ = this->create_subscription<SystemStatusMsg>(
    "system_status", 10, std::bind(&SafetyManagerNode::SystemStatusCB, this, _1));

  // -------------------------------
  //   Timers
  // -------------------------------

  const auto timer_freq = this->get_parameter("timer_frequency").as_double();
  const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

  safety_tree_timer_ = this->create_wall_timer(
    timer_period, std::bind(&SafetyManagerNode::SafetyTreeTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void SafetyManagerNode::DeclareParameters()
{
  const auto panther_manager_pkg_path =
    ament_index_cpp::get_package_share_directory("panther_manager");
  const auto default_bt_project_path = panther_manager_pkg_path +
                                       "/behavior_trees/PantherSafetyBT.btproj";
  const std::vector<std::string> default_plugin_libs = {};

  this->declare_parameter<std::string>("bt_project_path", default_bt_project_path);
  this->declare_parameter<std::string>("shutdown_hosts_path", "");
  this->declare_parameter<std::vector<std::string>>("plugin_libs", default_plugin_libs);
  this->declare_parameter<std::vector<std::string>>("ros_plugin_libs", default_plugin_libs);
  this->declare_parameter<double>("ros_communication_timeout.availability", 1.0);
  this->declare_parameter<double>("ros_communication_timeout.response", 1.0);

  this->declare_parameter<int>("battery.temp.window_len", 6);
  this->declare_parameter<int>("cpu.temp.window_len", 6);
  this->declare_parameter<float>("cpu.temp.fan_on", 70.0);
  this->declare_parameter<float>("cpu.temp.fan_off", 60.0);
  this->declare_parameter<int>("driver.temp.window_len", 6);
  this->declare_parameter<float>("driver.temp.fan_on", 45.0);
  this->declare_parameter<float>("driver.temp.fan_off", 35.0);
  this->declare_parameter<float>("timer_frequency", 10.0);
  this->declare_parameter<float>("fan_turn_off_timeout", 60.0);
}

void SafetyManagerNode::RegisterBehaviorTree()
{
  const auto bt_project_path = this->get_parameter("bt_project_path").as_string();

  const auto plugin_libs = this->get_parameter("plugin_libs").as_string_array();
  const auto ros_plugin_libs = this->get_parameter("ros_plugin_libs").as_string_array();

  const auto service_availability_timeout =
    this->get_parameter("ros_communication_timeout.availability").as_double();
  const auto service_response_timeout =
    this->get_parameter("ros_communication_timeout.response").as_double();

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

std::map<std::string, std::any> SafetyManagerNode::CreateSafetyInitialBlackboard()
{
  const float cpu_fan_on_temp = this->get_parameter("cpu.temp.fan_on").as_double();
  const float cpu_fan_off_temp = this->get_parameter("cpu.temp.fan_off").as_double();
  const float driver_fan_on_temp = this->get_parameter("driver.temp.fan_on").as_double();
  const float driver_fan_off_temp = this->get_parameter("driver.temp.fan_off").as_double();
  const float fan_turn_off_timeout = this->get_parameter("fan_turn_off_timeout").as_double();

  const std::map<std::string, std::any> safety_initial_bb = {
    {"CPU_FAN_OFF_TEMP", cpu_fan_off_temp},
    {"CPU_FAN_ON_TEMP", cpu_fan_on_temp},
    {"DRIVER_FAN_OFF_TEMP", driver_fan_off_temp},
    {"DRIVER_FAN_ON_TEMP", driver_fan_on_temp},
    {"FAN_TURN_OFF_TIMEOUT", fan_turn_off_timeout},
    {"CRITICAL_BAT_TEMP", kCriticalBatteryTemp},
    {"FATAL_BAT_TEMP", kFatalBatteryTemp},
    // battery health constants
    {"POWER_SUPPLY_HEALTH_UNKNOWN", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN)},
    {"POWER_SUPPLY_HEALTH_GOOD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD)},
    {"POWER_SUPPLY_HEALTH_OVERHEAT", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT)},
    {"POWER_SUPPLY_HEALTH_DEAD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD)},
    {"POWER_SUPPLY_HEALTH_OVERVOLTAGE", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE)},
    {"POWER_SUPPLY_HEALTH_UNSPEC_FAILURE",
     unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)},
    {"POWER_SUPPLY_HEALTH_COLD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD)},
    {"POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE",
     unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE)},
    {"POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE",
     unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE)},
  };

  RCLCPP_INFO(this->get_logger(), "Blackboard created.");
  return safety_initial_bb;
}

void SafetyManagerNode::BatteryCB(const BatteryStateMsg::SharedPtr battery)
{
  const auto battery_status = battery->power_supply_status;
  const auto battery_health = battery->power_supply_health;
  safety_tree_manager_->GetBlackboard()->set<unsigned>("battery_status", battery_status);
  safety_tree_manager_->GetBlackboard()->set<unsigned>("battery_health", battery_health);

  // don't update battery temperature if unknown status
  if (
    battery_status != BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN &&
    battery_health != BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
    battery_temp_ma_->Roll(battery->temperature);
  }

  safety_tree_manager_->GetBlackboard()->set<double>("bat_temp", battery_temp_ma_->GetAverage());
}

void SafetyManagerNode::DriverStateCB(const DriverStateMsg::SharedPtr driver_state)
{
  front_driver_temp_ma_->Roll(driver_state->front.temperature);
  rear_driver_temp_ma_->Roll(driver_state->rear.temperature);

  // to simplify conditions pass only higher temp of motor drivers
  safety_tree_manager_->GetBlackboard()->set<double>(
    "driver_temp",
    std::max({front_driver_temp_ma_->GetAverage(), rear_driver_temp_ma_->GetAverage()}));
}

void SafetyManagerNode::EStopCB(const BoolMsg::SharedPtr e_stop)
{
  safety_tree_manager_->GetBlackboard()->set<bool>("e_stop_state", e_stop->data);
}

void SafetyManagerNode::IOStateCB(const IOStateMsg::SharedPtr io_state)
{
  if (io_state->power_button) {
    ShutdownRobot("Power button pressed");
  }

  safety_tree_manager_->GetBlackboard()->set<bool>("aux_state", io_state->aux_power);
  safety_tree_manager_->GetBlackboard()->set<bool>("fan_state", io_state->fan);
}

void SafetyManagerNode::SystemStatusCB(const SystemStatusMsg::SharedPtr system_status)
{
  cpu_temp_ma_->Roll(system_status->cpu_temp);
  safety_tree_manager_->GetBlackboard()->set<double>("cpu_temp", cpu_temp_ma_->GetAverage());
}

void SafetyManagerNode::SafetyTreeTimerCB()
{
  if (!SystemReady()) {
    return;
  }

  safety_tree_manager_->TickOnce();

  if (safety_tree_manager_->GetTreeStatus() == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Safety behavior tree returned FAILURE status");
  }

  std::pair<bool, std::string> signal_shutdown;
  if (safety_tree_manager_->GetBlackboard()->get<std::pair<bool, std::string>>(
        "signal_shutdown", signal_shutdown)) {
    if (signal_shutdown.first) {
      ShutdownRobot(signal_shutdown.second);
    }
  }
}

bool SafetyManagerNode::SystemReady()
{
  if (
    !safety_tree_manager_->GetBlackboard()->getEntry("e_stop_state") ||
    !safety_tree_manager_->GetBlackboard()->getEntry("battery_status") ||
    !safety_tree_manager_->GetBlackboard()->getEntry("aux_state") ||
    !safety_tree_manager_->GetBlackboard()->getEntry("cpu_temp") ||
    !safety_tree_manager_->GetBlackboard()->getEntry("driver_temp")) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for required system messages to arrive.");
    return false;
  }

  return true;
}

void SafetyManagerNode::ShutdownRobot(const std::string & reason)
{
  RCLCPP_WARN(this->get_logger(), "Soft shutdown initialized. %s", reason.c_str());
  safety_tree_timer_->cancel();
  safety_tree_manager_->HaltTree();

  // tick shutdown tree
  auto status = BT::NodeStatus::RUNNING;
  auto start_time = this->get_clock()->now();
  rclcpp::Rate rate(30.0);  // 30 Hz
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    shutdown_tree_manager_->TickOnce();
    status = shutdown_tree_manager_->GetTreeStatus();
    rate.sleep();
  }

  if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(
      this->get_logger(),
      "Shutdown behavior tree returned FAILURE status, robot may not be shutdown correctly");
  }

  rclcpp::shutdown();
}

}  // namespace panther_manager
