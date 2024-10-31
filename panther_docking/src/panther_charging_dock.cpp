// Copyright (c) 2024 Husarion Sp. z o.o.
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

#include "panther_docking/panther_charging_dock.hpp"

#include <stdexcept>

#include <nav2_util/node_utils.hpp>

#include "panther_utils/common_utilities.hpp"
#include "panther_utils/tf2_utils.hpp"

namespace panther_docking
{

void PantherChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;

  if (!tf) {
    throw std::runtime_error("PantherChargingDock requires a TF buffer");
  }

  tf2_buffer_ = tf;

  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  declareParameters(node);
  getParameters(node);

  pose_filter_ = std::make_unique<opennav_docking::PoseFilter>(
    pose_filter_coef_, external_detection_timeout_);
}

void PantherChargingDock::cleanup()
{
  dock_pose_sub_.reset();
  staging_pose_pub_.reset();
}

void PantherChargingDock::activate()
{
  auto node = node_.lock();
  dock_pose_sub_ = node->create_subscription<PoseStampedMsg>(
    "docking/dock_pose", 1,
    std::bind(&PantherChargingDock::setDockPose, this, std::placeholders::_1));
  staging_pose_pub_ = node->create_publisher<PoseStampedMsg>("docking/staging_pose", 1);
}

void PantherChargingDock::deactivate()
{
  dock_pose_sub_.reset();
  staging_pose_pub_.reset();
}

void PantherChargingDock::declareParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  nav2_util::declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue("base_link"));

  nav2_util::declare_parameter_if_not_declared(node, "fixed_frame", rclcpp::ParameterValue("odom"));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".external_detection_timeout", rclcpp::ParameterValue(0.0));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".docking_distance_threshold", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".docking_yaw_threshold", rclcpp::ParameterValue(0.3));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".staging_x_offset", rclcpp::ParameterValue(-0.7));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".filter_coef", rclcpp::ParameterValue(0.1));
}

void PantherChargingDock::getParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  node->get_parameter("base_frame", base_frame_name_);
  node->get_parameter("fixed_frame", fixed_frame_name_);

  node->get_parameter(name_ + ".external_detection_timeout", external_detection_timeout_);
  node->get_parameter(name_ + ".docking_distance_threshold", docking_distance_threshold_);
  node->get_parameter(name_ + ".docking_yaw_threshold", docking_yaw_threshold_);
  node->get_parameter(name_ + ".staging_x_offset", staging_x_offset_);

  node->get_parameter(name_ + ".filter_coef", pose_filter_coef_);
}

// When there is no pose actual position of robot is a staging pose
PantherChargingDock::PoseStampedMsg PantherChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  RCLCPP_DEBUG_STREAM(logger_, "Getting staging pose in frame: " << frame);

  // When there is no global pose to reach thanks to nav2
  if (pose == geometry_msgs::msg::Pose()) {
    if (dock_pose_.header.frame_id.empty()) {
      throw opennav_docking_core::FailedToDetectDock("No dock pose detected");
    }

    updateAndPublishStagingPose();
  }

  return staging_pose_;
}

bool PantherChargingDock::getRefinedPose(PoseStampedMsg & pose)
{
  RCLCPP_DEBUG(logger_, "Getting refined pose");
  rclcpp::Time request_detection_time;

  if (dock_pose_.header.frame_id.empty()) {
    throw opennav_docking_core::FailedToDetectDock("No dock pose detected");
  }

  {
    auto node = node_.lock();
    request_detection_time = node->now();
  }

  auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
  auto duration = rclcpp::Time(request_detection_time) - rclcpp::Time(dock_pose_.header.stamp);
  if (duration > timeout) {
    RCLCPP_WARN_STREAM(
      logger_, "Lost detection or did not detect: timeout exceeded: " << duration.seconds());
    return false;
  }

  pose = dock_pose_;
  updateAndPublishStagingPose();

  return true;
}

bool PantherChargingDock::isDocked()
{
  RCLCPP_DEBUG(logger_, "Checking if docked");
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_name_;

  robot_pose = panther_utils::tf2_utils::TransformPose(tf2_buffer_, robot_pose, fixed_frame_name_);

  return panther_utils::tf2_utils::ArePosesNear(
    robot_pose, dock_pose_, docking_distance_threshold_, docking_yaw_threshold_);
}

bool PantherChargingDock::isCharging()
{
  RCLCPP_DEBUG(logger_, "Checking if charging");
  try {
    return isDocked();
  } catch (const opennav_docking_core::FailedToDetectDock & e) {
    return false;
  }
}

bool PantherChargingDock::disableCharging() { return true; }

bool PantherChargingDock::hasStoppedCharging() { return !isCharging(); }

void PantherChargingDock::setDockPose(const PoseStampedMsg::SharedPtr pose)
{
  auto filtered_pose = pose_filter_->update(*pose);
  dock_pose_ = filtered_pose;
}

void PantherChargingDock::updateAndPublishStagingPose()
{
  const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
  staging_pose_ = dock_pose_;
  staging_pose_.pose.position.x += cos(yaw) * staging_x_offset_;
  staging_pose_.pose.position.y += sin(yaw) * staging_x_offset_;

  staging_pose_pub_->publish(staging_pose_);
}

}  // namespace panther_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(panther_docking::PantherChargingDock, opennav_docking_core::ChargingDock)
