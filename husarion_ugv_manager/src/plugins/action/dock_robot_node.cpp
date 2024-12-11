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

#include "panther_manager/plugins/action/dock_robot_node.hpp"
#include "panther_manager/behavior_tree_utils.hpp"

namespace panther_manager
{

bool DockRobot::setGoal(Goal & goal)
{
  if (!this->getInput<std::string>("dock_type", goal.dock_type) || goal.dock_type.empty()) {
    RCLCPP_ERROR_STREAM(
      this->logger(), GetLoggerPrefix(name()) << "Failed to get input [dock_type]");
    return false;
  }

  if (!this->getInput<bool>("use_dock_id", goal.use_dock_id)) {
    RCLCPP_WARN_STREAM(
      this->logger(), GetLoggerPrefix(name())
                        << "use_dock_id not set, using default value: " << goal.use_dock_id);
  }

  if (
    (!this->getInput<std::string>("dock_id", goal.dock_id) || goal.dock_id.empty()) &&
    goal.use_dock_id) {
    RCLCPP_ERROR_STREAM(this->logger(), GetLoggerPrefix(name()) << "Failed to get input [dock_id]");
    return false;
  }

  if (!this->getInput<bool>("navigate_to_staging_pose", goal.navigate_to_staging_pose)) {
    RCLCPP_WARN_STREAM(
      this->logger(), GetLoggerPrefix(name()) << "navigate_to_staging_pose not set, using default "
                                                 "value: "
                                              << goal.navigate_to_staging_pose);
  }

  if (
    !this->getInput<float>("max_staging_time", goal.max_staging_time) &&
    goal.navigate_to_staging_pose) {
    RCLCPP_ERROR_STREAM(
      this->logger(), GetLoggerPrefix(name()) << "Failed to get input [max_staging_time]");
    return false;
  }

  return true;
}

BT::NodeStatus DockRobot::onResultReceived(const WrappedResult & wr)
{
  const auto & result = wr.result;

  this->setOutput("error_code", result->error_code);
  this->setOutput("num_retries", result->num_retries);

  if (result->success) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DockRobot::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR_STREAM(
    this->logger(), GetLoggerPrefix(name()) << ": onFailure with error: " << toStr(error));
  return BT::NodeStatus::FAILURE;
}

void DockRobot::onHalt()
{
  RCLCPP_INFO_STREAM(this->logger(), GetLoggerPrefix(name()) << ": onHalt");
}

}  // namespace panther_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(panther_manager::DockRobot, "DockRobot");
