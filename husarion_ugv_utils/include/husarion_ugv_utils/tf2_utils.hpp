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

#ifndef PANTHER_UTILS_TF2_UTILS_HPP_
#define PANTHER_UTILS_TF2_UTILS_HPP_

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace panther_utils::tf2_utils
{

/**
 * @brief Transforms a pose to a target frame.
 *  This function transforms a pose to a target frame using the provided tf2 buffer.
 *
 * @param tf2_buffer The tf2 buffer to be used for the transformation.
 * @param pose The pose to be transformed.
 * @param target_frame The target frame to which the pose should be transformed.
 * @param timeout_s The timeout in seconds for the transformation.
 *
 * @return The transformed pose.
 */
geometry_msgs::msg::PoseStamped TransformPose(
  const tf2_ros::Buffer::SharedPtr & tf2_buffer, const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame, double timeout_s = 0.0)
{
  geometry_msgs::msg::PoseStamped transformed_pose;

  if (pose.header.frame_id.empty() || target_frame.empty()) {
    throw std::runtime_error(
      "Pose or target frame is empty, pose frame: \"" + pose.header.frame_id +
      "\", target frame: \"" + target_frame + "\"");
  }

  if (!tf2_buffer->canTransform(
        pose.header.frame_id, target_frame, pose.header.stamp,
        rclcpp::Duration::from_seconds(timeout_s))) {
    throw std::runtime_error(
      "Cannot transform " + pose.header.frame_id + " to " + target_frame + " at time " +
      std::to_string(pose.header.stamp.sec) + "." + std::to_string(pose.header.stamp.nanosec));
  }

  tf2_buffer->transform(pose, transformed_pose, target_frame);

  return transformed_pose;
}

/**
 * @brief Offsets a pose by a given transform.
 * This function offsets a pose by a given transform in the same frame.
 *
 * @param pose The pose to be offset.
 * @param offset The offset transform.
 *
 * @return The offset pose.
 */
geometry_msgs::msg::PoseStamped OffsetPose(
  const geometry_msgs::msg::PoseStamped & pose, const tf2::Transform & offset)
{
  tf2::Transform pose_transform;
  tf2::fromMsg(pose.pose, pose_transform);

  tf2::Transform offset_pose_transform = pose_transform * offset;
  geometry_msgs::msg::PoseStamped transformed_pose;
  transformed_pose.header = pose.header;
  tf2::toMsg(offset_pose_transform, transformed_pose.pose);

  return transformed_pose;
}

/**
 * @brief Gets the roll, pitch, and yaw from a quaternion.
 * This function gets the roll, pitch, and yaw from a quaternion.
 * Roll is rotation around the X axis. Pitch is rotation around the Y axis. Yaw is rotation around
 * the Z axis.
 *
 * @param orientation The orientation quaternion.
 * @return geometry_msgs::msg::Vector3 The roll, pitch, and yaw.
 */
geometry_msgs::msg::Vector3 GetRPY(const geometry_msgs::msg::Quaternion & orientation)
{
  geometry_msgs::msg::Vector3 v;
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  tf2::Matrix3x3(q).getRPY(v.x, v.y, v.z);
  return v;
}

/**
 * @brief Checks if two poses are near each other on the XY plane.
 * This function checks if two poses are near each other on the XY plane within a given distance and
 * angle tolerance.
 *
 * @param pose_1 The first pose.
 * @param pose_2 The second pose.
 * @param distance_tolerance The distance tolerance.
 * @param angle_tolerance The angle tolerance.
 *
 * @return True if the poses are near each other on the XY plane, false otherwise.
 */
bool ArePosesNear(
  const geometry_msgs::msg::PoseStamped & pose_1, const geometry_msgs::msg::PoseStamped & pose_2,
  double distance_tolerance, double angle_tolerance)
{
  if (pose_1.header.frame_id.empty() || pose_2.header.frame_id.empty()) {
    throw std::runtime_error("Provided frame IDs are empty, can't compare poses.");
  }

  if (pose_1.header.frame_id != pose_2.header.frame_id) {
    throw std::runtime_error("Provided frame IDs are different, can't compare poses.");
  }

  auto pose_1_rpy = GetRPY(pose_1.pose.orientation);
  auto pose_2_rpy = GetRPY(pose_2.pose.orientation);

  const double d = std::hypot(
    pose_1.pose.position.x - pose_2.pose.position.x,
    pose_1.pose.position.y - pose_2.pose.position.y);
  return d < distance_tolerance && std::abs(pose_1_rpy.x - pose_2_rpy.x) < angle_tolerance &&
         std::abs(pose_1_rpy.y - pose_2_rpy.y) < angle_tolerance &&
         std::abs(pose_1_rpy.z - pose_2_rpy.z) < angle_tolerance;
}
}  // namespace panther_utils::tf2_utils

#endif  // PANTHER_UTILS_TF2_UTILS_HPP_
