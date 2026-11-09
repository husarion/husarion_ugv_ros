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

#include "husarion_ugv_manager/safety_manager_node.hpp"

#include <algorithm>
#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_utils/moving_average.hpp"

#include "husarion_ugv_manager/behavior_tree_manager.hpp"
#include "husarion_ugv_manager/behavior_tree_utils.hpp"
#include "husarion_ugv_manager/safety_manager_parameters.hpp"

namespace husarion_ugv_manager
{

static_assert(UvpReactionInput::kStateUvp == BatteryProtectionMsg::STATE_UVP);
static_assert(UvpReactionInput::kModeStop == BatteryProtectionMsg::MODE_STOP);
static_assert(UvpReactionInput::kModeShutdown == BatteryProtectionMsg::MODE_SHUTDOWN);

SafetyManagerNode::SafetyManagerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  this->param_listener_ =
    std::make_shared<safety_manager::ParamListener>(this->get_node_parameters_interface());
  this->params_ = this->param_listener_->get_params();

  const auto battery_temp_window_len = this->params_.battery.temp.window_len;
  const auto cpu_temp_window_len = this->params_.cpu.temp.window_len;

  battery_temp_ma_ =
    std::make_unique<husarion_ugv_utils::MovingAverage<double>>(battery_temp_window_len);
  cpu_temp_ma_ = std::make_unique<husarion_ugv_utils::MovingAverage<double>>(cpu_temp_window_len);

  const auto safety_initial_blackboard = CreateSafetyInitialBlackboard();
  safety_tree_manager_ = std::make_unique<BehaviorTreeManager>(
    "Safety", safety_initial_blackboard, 6666, this->params_.groot_enabled);

  const auto shutdown_hosts_path = this->params_.shutdown_hosts_path;
  const std::map<std::string, std::any> shutdown_initial_blackboard = {
    {"SHUTDOWN_HOSTS_FILE", shutdown_hosts_path},
    {"SHUTDOWN_LOCALHOST_COMMAND", kShutdownLocalhostCommand},
  };
  shutdown_tree_manager_ = std::make_unique<BehaviorTreeManager>(
    "Shutdown", shutdown_initial_blackboard, 7777, this->params_.groot_enabled);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void SafetyManagerNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  RegisterBehaviorTree();
  shutdown_tree_manager_->Initialize(factory_);

  const auto startup_timeout = this->params_.ros_communication_timeout.startup;
  if (startup_timeout > 0.0 && !this->params_.startup_services.empty()) {
    // The shutdown tree does not wait: a power button press during the wait still shuts down.
    auto io_state_sub = this->create_subscription<IOStateMsg>(
      "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&SafetyManagerNode::CheckPowerButton, this, std::placeholders::_1));
    // The hardware interface creates e_stop_trigger after aux_power_enable and fan_enable, so the
    // default service list normally covers every service of the default safety tree.
    behavior_tree_utils::WaitForServices(
      this->shared_from_this(), this->params_.startup_services,
      std::chrono::duration<double>(startup_timeout));
    // A power button press or a signal during the wait has shut the context down.
    if (!this->get_node_base_interface()->get_context()->is_valid()) {
      return;
    }
  }

  safety_tree_manager_->Initialize(factory_);

  if (this->params_.uvp_enable) {
    InitializeUVP();
  }

  // -------------------------------
  //   Subscribers
  // -------------------------------

  using namespace std::placeholders;

  battery_sub_ = this->create_subscription<BatteryStateMsg>(
    "battery/battery_status", 10, std::bind(&SafetyManagerNode::BatteryCB, this, _1));
  driver_state_sub_ = this->create_subscription<RobotDriverStateMsg>(
    "hardware/robot_driver_state", 10, std::bind(&SafetyManagerNode::RobotDriverStateCB, this, _1));
  e_stop_sub_ = this->create_subscription<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SafetyManagerNode::EStopCB, this, _1));
  io_state_sub_ = this->create_subscription<IOStateMsg>(
    "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SafetyManagerNode::IOStateCB, this, _1));
  system_status_sub_ = this->create_subscription<SystemStatusMsg>(
    "system_status", 10, std::bind(&SafetyManagerNode::SystemStatusCB, this, _1));

  const double timer_freq = this->params_.timer_frequency;
  const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

  safety_tree_timer_ = this->create_wall_timer(
    timer_period, std::bind(&SafetyManagerNode::SafetyTreeTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void SafetyManagerNode::InitializeUVP()
{
  uvp_reaction_ = std::make_unique<UvpReaction>();
  uvp_start_ns_ = SteadyNowNs();

  try {
    factory_.registerBehaviorTreeFromText(kUVPShutdownTreeXML);
    const std::map<std::string, std::any> uvp_shutdown_initial_blackboard = {
      {"SHUTDOWN_HOSTS_FILE", this->params_.shutdown_hosts_path},
      {"UVP_HOSTS_TIMEOUT",
       UvpReaction::ShutdownHostsTimeoutS(uvp_input_.shutdown_hosts_timeout_s)},
    };
    uvp_shutdown_tree_manager_ = std::make_unique<BehaviorTreeManager>(
      "UVPShutdown", uvp_shutdown_initial_blackboard, 8888, this->params_.groot_enabled);
    uvp_shutdown_tree_manager_->Initialize(factory_);
  } catch (const std::exception & e) {
    // The plugins it needs come from the plugin_libs parameter, which a user can change.
    uvp_shutdown_tree_manager_.reset();
    uvp_shutdown_tree_error_ = e.what();
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "UVP shutdown sequence unavailable, a UVP shutdown uses the normal "
      "shutdown tree: "
        << e.what());
  }

  aux_power_client_ = this->create_client<SetBoolSrv>("hardware/aux_power_enable");

  using namespace std::placeholders;
  protection_sub_ = this->create_subscription<BatteryProtectionMsg>(
    "battery/protection", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SafetyManagerNode::ProtectionCB, this, _1));

  // T keeps running when the battery driver stops publishing, see UvpReaction for stale data.
  uvp_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&SafetyManagerNode::UVPTimerCB, this));

  diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  diagnostic_updater_->setHardwareID("Safety manager");
  diagnostic_updater_->add("Battery protection reaction", this, &SafetyManagerNode::DiagnoseUVP);

  RCLCPP_INFO(this->get_logger(), "Battery undervoltage protection reactions enabled.");
}

void SafetyManagerNode::RegisterBehaviorTree()
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

std::map<std::string, std::any> SafetyManagerNode::CreateSafetyInitialBlackboard()
{
  const double cpu_fan_on_temp = this->params_.cpu.temp.fan_on;
  const double cpu_fan_off_temp = this->params_.cpu.temp.fan_off;
  const double driver_fan_on_temp = this->params_.driver.temp.fan_on;
  const double driver_fan_off_temp = this->params_.driver.temp.fan_off;
  const float fan_turn_off_timeout = static_cast<float>(this->params_.fan_turn_off_timeout);

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

void SafetyManagerNode::RobotDriverStateCB(const RobotDriverStateMsg::SharedPtr driver_state)
{
  if (driver_state->driver_states.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty driver state message.");
    return;
  }

  for (auto & driver : driver_state->driver_states) {
    if (driver_temp_ma_.find(driver.name) == driver_temp_ma_.end()) {
      RCLCPP_DEBUG(
        this->get_logger(), "Creating moving average for driver '%s'", driver.name.c_str());
      const auto driver_temp_window_len = this->params_.driver.temp.window_len;
      driver_temp_ma_[driver.name] =
        std::make_unique<husarion_ugv_utils::MovingAverage<double>>(driver_temp_window_len);
    }

    driver_temp_ma_[driver.name]->Roll(driver.state.temperature);
  }

  // to simplify conditions pass only higher temp of motor drivers
  const auto max_element = std::max_element(
    driver_temp_ma_.begin(), driver_temp_ma_.end(),
    [](const auto & a, const auto & b) { return a.second->GetAverage() < b.second->GetAverage(); });

  safety_tree_manager_->GetBlackboard()->set<double>(
    "driver_temp", max_element->second->GetAverage());
}

void SafetyManagerNode::EStopCB(const BoolMsg::SharedPtr e_stop)
{
  safety_tree_manager_->GetBlackboard()->set<bool>("e_stop_state", e_stop->data);
}

void SafetyManagerNode::CheckPowerButton(const IOStateMsg::SharedPtr io_state)
{
  if (io_state->power_button) {
    ShutdownRobot("Power button pressed");
  }
}

void SafetyManagerNode::IOStateCB(const IOStateMsg::SharedPtr io_state)
{
  CheckPowerButton(io_state);

  safety_tree_manager_->GetBlackboard()->set<bool>("aux_state", io_state->aux_power);
  safety_tree_manager_->GetBlackboard()->set<bool>("fan_state", io_state->fan);
}

void SafetyManagerNode::SystemStatusCB(const SystemStatusMsg::SharedPtr system_status)
{
  cpu_temp_ma_->Roll(system_status->cpu_temp);
  safety_tree_manager_->GetBlackboard()->set<double>("cpu_temp", cpu_temp_ma_->GetAverage());
}

void SafetyManagerNode::ProtectionCB(const BatteryProtectionMsg::SharedPtr msg)
{
  uvp_input_.received = true;
  uvp_input_.state = msg->state;
  uvp_input_.mode = msg->mode;
  uvp_input_.reactions_enabled = msg->reactions_enabled;
  uvp_input_.stop_timeout_min = msg->stop_timeout_min;
  uvp_input_.aux_off_at_trip = msg->aux_off_at_trip;
  uvp_input_.shutdown_hosts_timeout_s = msg->shutdown_hosts_timeout_s;
  uvp_input_.trip_id = msg->trip_id;
  uvp_input_.trip_stamp_ns = rclcpp::Time(msg->trip_stamp).nanoseconds();
  uvp_input_.seconds_without_charging = msg->seconds_without_charging;
  uvp_input_.charging = msg->charging;
  uvp_input_.charger_connected = msg->charger_connected;
  uvp_reactions_disabled_reason_ = msg->reactions_disabled_reason;
  uvp_last_msg_ns_ = SteadyNowNs();
}

void SafetyManagerNode::UVPTimerCB()
{
  const auto now_ns = SteadyNowNs();
  if (uvp_input_.received) {
    uvp_input_.sample_age_s = static_cast<double>(now_ns - uvp_last_msg_ns_) / 1e9;
  }
  const auto actions = uvp_reaction_->Update(uvp_input_, now_ns);

  if (actions.aux_off) {
    RCLCPP_WARN(this->get_logger(), "Battery undervoltage protection: turning the AUX power off.");
    uvp_aux_off_pending_ = true;
  }
  if (!uvp_reaction_->Latched()) {
    uvp_aux_off_pending_ = false;
  }
  if (uvp_aux_off_in_flight_ && now_ns - uvp_aux_off_sent_ns_ > UVPAuxOffResponseTimeoutNs()) {
    // A hardware system that went away after the request never answers it.
    aux_power_client_->remove_pending_request(uvp_aux_off_request_id_);
    uvp_aux_off_in_flight_ = false;
    RCLCPP_WARN(
      this->get_logger(),
      "Battery undervoltage protection: no answer to the AUX power off, trying again.");
  }
  if (uvp_aux_off_pending_ && !uvp_aux_off_in_flight_ && now_ns >= uvp_aux_off_next_try_ns_) {
    SendUVPAuxOff();
  }

  if (actions.shutdown) {
    ShutdownRobot(actions.reason, true);
  }
}

void SafetyManagerNode::SendUVPAuxOff()
{
  // A request sent before the service is matched is lost, so it waits for the hardware system,
  // which may start after a trip at boot.
  if (!aux_power_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000,
      "Battery undervoltage protection: waiting for hardware/aux_power_enable to turn the AUX "
      "power off.");
    return;
  }

  auto request = std::make_shared<SetBoolSrv::Request>();
  request->data = false;
  uvp_aux_off_in_flight_ = true;
  uvp_aux_off_sent_ns_ = SteadyNowNs();
  const auto future_and_id = aux_power_client_->async_send_request(
    request, [this](rclcpp::Client<SetBoolSrv>::SharedFuture future) {
      uvp_aux_off_in_flight_ = false;
      const auto response = future.get();
      if (response->success) {
        uvp_aux_off_pending_ = false;
        uvp_aux_off_backoff_ns_ = kUVPAuxOffMinBackoffNs;
        RCLCPP_INFO(this->get_logger(), "Battery undervoltage protection: AUX power off.");
        return;
      }
      // A refusal is not retried at the timer rate: the wait doubles up to a cap.
      uvp_aux_off_next_try_ns_ = SteadyNowNs() + uvp_aux_off_backoff_ns_;
      uvp_aux_off_backoff_ns_ = std::min(2 * uvp_aux_off_backoff_ns_, kUVPAuxOffMaxBackoffNs);
      RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "Failed to turn the AUX power off, trying again: " << response->message);
    });
  uvp_aux_off_request_id_ = future_and_id.request_id;
}

std::int64_t SafetyManagerNode::UVPAuxOffResponseTimeoutNs() const
{
  // The response timeout of the behavior tree service calls.
  return static_cast<std::int64_t>(this->params_.ros_communication_timeout.response * 1e9);
}

std::int64_t SafetyManagerNode::SteadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

bool SafetyManagerNode::UseUVPShutdown(const bool uvp_requested) const
{
  return uvp_shutdown_tree_manager_ &&
         (uvp_requested || (uvp_reaction_ && uvp_reaction_->Latched()));
}

void SafetyManagerNode::DiagnoseUVP(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char level = diagnostic_updater::DiagnosticStatusWrapper::OK;
  std::string message = "Battery protection not tripped.";

  const auto now_ns = SteadyNowNs();

  status.add("UVP", uvp_reaction_->Latched() ? std::string("latched") : std::string("not latched"));
  const auto seconds_to_shutdown = uvp_reaction_->SecondsToShutdown(now_ns);
  if (seconds_to_shutdown.has_value()) {
    status.add("Seconds to UVP shutdown", seconds_to_shutdown.value());
  }
  if (uvp_reaction_->Latched()) {
    level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
    message = seconds_to_shutdown.has_value()
                ? "Battery undervoltage: the robot shuts down unless it charges."
                : "Battery undervoltage: the shutdown waits while the robot charges.";
  } else if (
    uvp_input_.received && uvp_input_.state == UvpReactionInput::kStateUvp &&
    !uvp_input_.reactions_enabled) {
    level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
    message = "Battery undervoltage, reactions disabled: " + uvp_reactions_disabled_reason_;
  }
  status.add(
    "UVP shutdown sequence",
    uvp_shutdown_tree_manager_ ? std::string("ready") : "unavailable: " + uvp_shutdown_tree_error_);

  if (uvp_last_msg_ns_ == 0) {
    status.add("UVP data", "not received yet");
    if (static_cast<double>(now_ns - uvp_start_ns_) / 1e9 > kUVPTopicStartupGrace) {
      level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
      message = "No battery protection data on battery/protection, the protection cannot act.";
    }
  } else {
    const auto age = static_cast<double>(now_ns - uvp_last_msg_ns_) / 1e9;
    status.add("UVP data age (s)", age);
    if (age > UvpReaction::kDefaultStaleTimeoutS) {
      level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
      message = uvp_reaction_->Latched()
                  ? "Battery protection data is stale, charging no longer holds the shutdown."
                  : "Battery protection data is stale.";
    }
  }

  // Last, so no other message hides it.
  if (!uvp_shutdown_tree_manager_) {
    level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "UVP shutdown sequence unavailable, a UVP shutdown uses the normal shutdown tree.";
  }

  status.summary(level, message);
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

void SafetyManagerNode::ShutdownRobot(const std::string & reason, const bool uvp_requested)
{
  const bool uvp_sequence = UseUVPShutdown(uvp_requested);
  const std::string sequence = uvp_sequence ? " (UVP shutdown sequence)" : "";
  RCLCPP_WARN(
    this->get_logger(), "Soft shutdown initialized. %s%s", reason.c_str(), sequence.c_str());

  // The safety tree does not exist yet when the shutdown comes during the startup wait.
  if (safety_tree_timer_) {
    safety_tree_timer_->cancel();
    safety_tree_manager_->HaltTree();
  }
  if (uvp_timer_) {
    uvp_timer_->cancel();
  }

  // The UVP part runs in front of the Shutdown tree, which then runs as always, a user's own
  // included: the user computers get a capped wait and the AUX power goes off first.
  if (uvp_sequence) {
    uvp_shutdown_tree_manager_->GetBlackboard()->set<float>(
      "UVP_HOSTS_TIMEOUT", UvpReaction::ShutdownHostsTimeoutS(uvp_input_.shutdown_hosts_timeout_s));
    TickUntilDone(*uvp_shutdown_tree_manager_);
  }

  const auto status = TickUntilDone(*shutdown_tree_manager_);
  if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(
      this->get_logger(),
      "Shutdown behavior tree returned FAILURE status, robot may not be shutdown correctly");
  }

  StopNode();
}

void SafetyManagerNode::StopNode() { rclcpp::shutdown(); }

BT::NodeStatus SafetyManagerNode::TickUntilDone(BehaviorTreeManager & tree_manager)
{
  auto status = BT::NodeStatus::RUNNING;
  rclcpp::Rate rate(30.0);  // 30 Hz
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    tree_manager.TickOnce();
    status = tree_manager.GetTreeStatus();
    rate.sleep();
  }
  return status;
}

}  // namespace husarion_ugv_manager
