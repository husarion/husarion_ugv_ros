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

#ifndef HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_SAFETY_MANAGER_NODE_HPP_
#define HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_SAFETY_MANAGER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "husarion_ugv_msgs/msg/battery_protection.hpp"
#include "husarion_ugv_msgs/msg/io_state.hpp"
#include "husarion_ugv_msgs/msg/robot_driver_state.hpp"
#include "husarion_ugv_msgs/msg/system_status.hpp"

#include "husarion_ugv_utils/moving_average.hpp"

#include "husarion_ugv_manager/behavior_tree_manager.hpp"
#include "husarion_ugv_manager/safety_manager_parameters.hpp"
#include "husarion_ugv_manager/uvp_reaction.hpp"

namespace husarion_ugv_manager
{

using BatteryProtectionMsg = husarion_ugv_msgs::msg::BatteryProtection;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using BoolMsg = std_msgs::msg::Bool;
using RobotDriverStateMsg = husarion_ugv_msgs::msg::RobotDriverState;
using IOStateMsg = husarion_ugv_msgs::msg::IOState;
using SystemStatusMsg = husarion_ugv_msgs::msg::SystemStatus;
using SetBoolSrv = std_srvs::srv::SetBool;

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

  // The battery undervoltage protection, only when uvp_enable is set.
  void InitializeUVP();
  // Whether a shutdown takes the UVP sequence: asked for, or any shutdown while UVP is latched.
  bool UseUVPShutdown(const bool uvp_requested) const;
  // uvp_requested: asked for by the protection, which takes the UVP sequence.
  void ShutdownRobot(const std::string & reason, const bool uvp_requested = false);
  // Ends the node once the shutdown tree is done. Virtual for the tests, which keep running.
  virtual void StopNode();
  void DiagnoseUVP(diagnostic_updater::DiagnosticStatusWrapper & status);

  // The hosts, each with its wait capped, then the AUX power. It runs in front of the Shutdown
  // tree, which powers this computer off. Kept in the node rather than in the user's behavior tree
  // project, which may predate it.
  static constexpr char kUVPShutdownTreeXML[] = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="UVPShutdown">
    <Sequence>
      <ForceSuccess>
        <ShutdownHostsFromFile shutdown_hosts_file="{SHUTDOWN_HOSTS_FILE}"
                               timeout_override="{UVP_HOSTS_TIMEOUT}"/>
      </ForceSuccess>
      <ForceSuccess>
        <CallSetBoolService service_name="hardware/aux_power_enable" data="false"/>
      </ForceSuccess>
    </Sequence>
  </BehaviorTree>
</root>
)";

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BehaviorTreeManager> safety_tree_manager_;
  std::unique_ptr<BehaviorTreeManager> shutdown_tree_manager_;
  std::unique_ptr<BehaviorTreeManager> uvp_shutdown_tree_manager_;
  std::unique_ptr<UvpReaction> uvp_reaction_;

private:
  void BatteryCB(const BatteryStateMsg::SharedPtr battery);
  void RobotDriverStateCB(const RobotDriverStateMsg::SharedPtr driver_state);
  void EStopCB(const BoolMsg::SharedPtr e_stop);
  void IOStateCB(const IOStateMsg::SharedPtr io_state);
  void SystemStatusCB(const SystemStatusMsg::SharedPtr system_status);
  void SafetyTreeTimerCB();
  void ProtectionCB(const BatteryProtectionMsg::SharedPtr msg);
  void UVPTimerCB();
  void SendUVPAuxOff();
  std::int64_t UVPAuxOffResponseTimeoutNs() const;
  BT::NodeStatus TickUntilDone(BehaviorTreeManager & tree_manager);
  // T runs on the steady clock, so simulation time cannot stretch it.
  static std::int64_t SteadyNowNs();

  void CheckPowerButton(const IOStateMsg::SharedPtr io_state);

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

  rclcpp::Subscription<BatteryProtectionMsg>::SharedPtr protection_sub_;
  rclcpp::TimerBase::SharedPtr uvp_timer_;
  rclcpp::Client<SetBoolSrv>::SharedPtr aux_power_client_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  UvpReactionInput uvp_input_;
  std::int64_t uvp_last_msg_ns_ = 0;
  std::int64_t uvp_start_ns_ = 0;
  std::string uvp_shutdown_tree_error_;
  std::string uvp_reactions_disabled_reason_;
  // The single-threaded executor runs the subscription, the timer and the service response, so
  // these need no lock.
  bool uvp_aux_off_pending_ = false;
  bool uvp_aux_off_in_flight_ = false;
  std::int64_t uvp_aux_off_request_id_ = 0;
  std::int64_t uvp_aux_off_sent_ns_ = 0;
  std::int64_t uvp_aux_off_next_try_ns_ = 0;
  static constexpr std::int64_t kUVPAuxOffMinBackoffNs = 1'000'000'000;
  static constexpr std::int64_t kUVPAuxOffMaxBackoffNs = 10'000'000'000;
  std::int64_t uvp_aux_off_backoff_ns_ = kUVPAuxOffMinBackoffNs;

  // Started together with the battery driver, which needs a few seconds before it publishes.
  static constexpr double kUVPTopicStartupGrace = 30.0;

  std::unique_ptr<husarion_ugv_utils::MovingAverage<double>> battery_temp_ma_;
  std::unique_ptr<husarion_ugv_utils::MovingAverage<double>> cpu_temp_ma_;

  std::map<std::string, std::unique_ptr<husarion_ugv_utils::MovingAverage<double>>> driver_temp_ma_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_SAFETY_MANAGER_NODE_HPP_
