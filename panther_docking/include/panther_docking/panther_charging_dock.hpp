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

#ifndef PANTHER_DOCKING_PANTHER_CHARGING_DOCK_HPP_
#define PANTHER_DOCKING_PANTHER_CHARGING_DOCK_HPP_

#include <memory>
#include <string>
#include <thread>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <opennav_docking/pose_filter.hpp>
#include <opennav_docking_core/charging_dock.hpp>
#include <opennav_docking_core/docking_exceptions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "wibotic_msgs/msg/wibotic_info.hpp"

namespace panther_docking
{

constexpr double kWiboticChargingCurrentThreshold = 0.0;

/**
 * @class PantherChargingDock
 * @brief A class to represent a Panther charging dock.
 */
class PantherChargingDock : public opennav_docking_core::ChargingDock
{
public:
  using SharedPtr = std::shared_ptr<PantherChargingDock>;
  using UniquePtr = std::unique_ptr<PantherChargingDock>;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using WiboticInfoMsg = wibotic_msgs::msg::WiboticInfo;

  /**
   * @brief Configure the dock with the necessary information.
   *
   * @param  parent Pointer to parent node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf) override final;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  void cleanup() override final;

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  void activate() override final;

  /**
   * @brief Method to deactivate Behavior and any threads involved in execution.
   */
  void deactivate() override final;

  /**
   * @brief Method to obtain the dock's staging pose. This method should likely
   * be using TF and the dock's pose information to find the staging pose from
   * a static or parameterized staging pose relative to the docking pose
   * @param pose Dock pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */
  PoseStampedMsg getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame) override final;

  /**
   * @brief Method to obtain the refined pose of the dock, usually based on sensors
   * @param pose The initial estimate of the dock pose.
   */
  bool getRefinedPose(PoseStampedMsg & pose) override final;

  /**
   * @brief Have we made contact with dock? This can be implemented in a variety
   * of ways: by establishing communications with the dock, by monitoring the
   * the drive motor efforts, etc.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  bool isDocked() override final;

  /**
   * @brief Are we charging? If a charge dock requires any sort of negotiation
   * to begin charging, that should happen inside this function as this function
   * will be called repeatedly after the docking loop to check if successful.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  bool isCharging() override final;

  /**
   * @brief Undocking while current is still flowing can damage a charge dock
   * so some charge docks provide the ability to disable charging before the
   * robot physically disconnects. The undocking action will not command the
   * robot to move until this returns true.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  bool disableCharging() override final;

  /**
   * @brief Similar to isCharging() but called when undocking.
   */
  bool hasStoppedCharging() override final;

protected:
  /**
   * @brief Method to declare parameters.
   *
   * @param node The node to declare parameters in.
   */
  void declareParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Method to get parameters.
   *
   * @param node The node to declare parameters in.
   */
  void getParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Method to update and publish the staging pose.
   *
   * Uses staging_x_offset_ and staging_yaw_offset_ to calculate the staging pose.
   */
  void updateAndPublishStagingPose();

  /**
   * @brief Dock pose callback, used for external detection.
   *
   * @param pose The dock pose.
   */
  void setDockPose(const PoseStampedMsg::SharedPtr pose);

  /**
   * @brief Wibotic info callback, used when `use_wibotic_info` is enabled.
   *
   * @param msg The Wibotic info message.
   */
  void setWiboticInfo(const WiboticInfoMsg::SharedPtr msg);

  std::string base_frame_name_;
  std::string fixed_frame_name_;
  std::string dock_frame_;

  rclcpp::Logger logger_{rclcpp::get_logger("PantherChargingDock")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  tf2_ros::Buffer::SharedPtr tf2_buffer_;

  rclcpp::Publisher<PoseStampedMsg>::SharedPtr staging_pose_pub_;
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr dock_pose_sub_;
  rclcpp::Subscription<WiboticInfoMsg>::SharedPtr wibotic_info_sub_;

  PoseStampedMsg dock_pose_;
  PoseStampedMsg staging_pose_;
  WiboticInfoMsg::SharedPtr wibotic_info_;

  double external_detection_timeout_;

  std::shared_ptr<opennav_docking::PoseFilter> pose_filter_;

  double docking_distance_threshold_;
  double docking_yaw_threshold_;

  double staging_x_offset_;
  double staging_yaw_offset_;

  double pose_filter_coef_;

  bool use_wibotic_info_;
  double wibotic_info_timeout_;
};

}  // namespace panther_docking

#endif  // PANTHER_DOCKING_PANTHER_CHARGING_DOCK_HPP_
