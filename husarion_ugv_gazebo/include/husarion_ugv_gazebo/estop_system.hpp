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

#ifndef HUSARION_UGV_GAZEBO_HUSARION_UGV_GAZEBO_ESTOP_SYSTEM_HPP_
#define HUSARION_UGV_GAZEBO_HUSARION_UGV_GAZEBO_ESTOP_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Element.hh>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace husarion_ugv_gazebo
{
using BoolMsg = std_msgs::msg::Bool;
using TriggerSrv = std_srvs::srv::Trigger;

/**
 * @brief Standalone E-stop simulation plugin. It exposes the same ROS interface as the robot
 * hardware (the e_stop topic and the trigger/reset services) and, while the E-stop is active,
 * overrides the joint velocity commands written by gz_ros2_control with zeros so the robot stops.
 *
 * It is a plain gz-sim system plugin (not a ros2_control hardware plugin), so the robot uses the
 * stock gz_ros2_control/GazeboSimSystem. This avoids the plugin-loader symbol clash that broke
 * loading GazeboSimSystem for other robots spawned after the UGV (issue #623).
 *
 * @note The plugin must be declared in SDF after the gz_ros2_control plugin - systems execute in
 * declaration order, and the zeros must be written after gz_ros2_control writes its commands.
 */
class EStopSystem : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
public:
  ~EStopSystem() override;

  /**
   * @brief Reads parameters, collects the model joints and sets up the ROS interface.
   *
   * @exception std::runtime_error if the plugin is not attached to a valid model.
   */
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override;

private:
  void SetupROSInterface();

  void PublishEStopStatus();

  void EStopResetCallback(
    const TriggerSrv::Request::SharedPtr & request, TriggerSrv::Response::SharedPtr response);

  void EStopTriggerCallback(
    const TriggerSrv::Request::SharedPtr & request, TriggerSrv::Response::SharedPtr response);

  std::atomic_bool e_stop_active_{true};
  std::string ns_ = "";
  std::vector<gz::sim::Entity> joint_entities_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<BoolMsg>::SharedPtr e_stop_publisher_;
  rclcpp::Service<TriggerSrv>::SharedPtr e_stop_reset_service_;
  rclcpp::Service<TriggerSrv>::SharedPtr e_stop_trigger_service_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
};

}  // namespace husarion_ugv_gazebo

#endif  // HUSARION_UGV_GAZEBO_HUSARION_UGV_GAZEBO_ESTOP_SYSTEM_HPP_
