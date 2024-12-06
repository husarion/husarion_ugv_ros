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

#include "panther_docking/dock_pose_publisher_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace panther_docking
{
DockPosePublisherNode::DockPosePublisherNode(const std::string & name)
: rclcpp_lifecycle::LifecycleNode(name)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPosePublisherNode::on_configure(const rclcpp_lifecycle::State &)
{
  declare_parameter("publish_rate", 10.0);
  declare_parameter("docks", std::vector<std::string>({"main"}));
  declare_parameter("fixed_frame", "odom");
  declare_parameter("base_frame", "base_link");
  declare_parameter("panther_charging_dock.external_detection_timeout", 0.1);

  const auto fixed_frame = get_parameter("fixed_frame").as_string();
  const auto docks = get_parameter("docks").as_string_array();
  const auto publish_rate = get_parameter("publish_rate").as_double();
  publish_period_ = std::chrono::duration<double>(1.0 / publish_rate);

  timeout_ = get_parameter("panther_charging_dock.external_detection_timeout").as_double() * 0.1;
  base_frame_ = get_parameter("base_frame").as_string();

  for (const auto & dock : docks) {
    declare_parameter(dock + ".type", "panther_charging_dock");
    declare_parameter(dock + ".dock_frame", "main_wibotic_transmitter_link");

    const auto dock_type = get_parameter(dock + ".type").as_string();
    if (dock_type == "panther_charging_dock") {
      const auto dock_pose_frame_id = get_parameter(dock + ".dock_frame").as_string();
      RCLCPP_INFO_STREAM(
        this->get_logger(), "Adding dock " << dock << " with frame " << dock_pose_frame_id);
      source_frames_.push_back(dock_pose_frame_id);
    }
  }
  target_frame_ = fixed_frame;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "docking/dock_pose", 10);

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Node configured successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPosePublisherNode::on_activate(const rclcpp_lifecycle::State &)
{
  pose_publisher_->on_activate();
  timer_ = this->create_wall_timer(
    publish_period_, std::bind(&DockPosePublisherNode::publishPose, this));

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPosePublisherNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  pose_publisher_->on_deactivate();
  timer_.reset();

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPosePublisherNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  pose_publisher_.reset();
  timer_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  source_frames_.clear();

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DockPosePublisherNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Node shutting down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void DockPosePublisherNode::publishPose()
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = target_frame_;

  geometry_msgs::msg::TransformStamped closest_dock;
  geometry_msgs::msg::TransformStamped base_transform_stamped;

  bool found = false;
  double closest_dist = std::numeric_limits<double>::max();

  try {
    base_transform_stamped = tf_buffer_->lookupTransform(
      target_frame_, base_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Could not get transform: " << ex.what());
    return;
  }

  for (const auto & source_frame : source_frames_) {
    try {
      const auto transform_stamped = tf_buffer_->lookupTransform(
        target_frame_, source_frame, tf2::TimePointZero, tf2::durationFromSec(timeout_));

      const double dist = std::hypot(
        transform_stamped.transform.translation.x - base_transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y - base_transform_stamped.transform.translation.y);

      if (dist < kMinimalDetectionDistance && dist < closest_dist) {
        closest_dist = dist;
        closest_dock = transform_stamped;
        found = true;
      }
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Could not get transform: " << ex.what());
      continue;
    }
  }

  if (!found) {
    RCLCPP_DEBUG(this->get_logger(), "No dock found");
    return;
  }

  pose_msg.pose.position.x = closest_dock.transform.translation.x;
  pose_msg.pose.position.y = closest_dock.transform.translation.y;
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation = closest_dock.transform.rotation;
  pose_publisher_->publish(pose_msg);
}

}  // namespace panther_docking
