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

#include "panther_manager/plugins/action/undock_robot_node.hpp"
#include "panther_manager/behavior_tree_utils.hpp"

namespace panther_manager
{

bool UndockRobot::setGoal(Goal & goal)
{
  if (!this->getInput<std::string>("dock_type", goal.dock_type) || goal.dock_type.empty()) {
    RCLCPP_ERROR_STREAM(
      this->logger(), GetLoggerPrefix(name()) << "Failed to get input [dock_type]");
    return false;
  }

  if (!this->getInput<float>("max_undocking_time", goal.max_undocking_time)) {
    RCLCPP_ERROR_STREAM(
      this->logger(), GetLoggerPrefix(name()) << "Failed to get input [max_undocking_time]");
    return false;
  }

  return true;
}

BT::NodeStatus UndockRobot::onResultReceived(const WrappedResult & wr)
{
  const auto & result = wr.result;

  this->setOutput("error_code", result->error_code);

  if (result->success) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus UndockRobot::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR_STREAM(
    this->logger(), GetLoggerPrefix(name()) << ": onFailure with error: " << toStr(error));
  return BT::NodeStatus::FAILURE;
}

void UndockRobot::onHalt()
{
  RCLCPP_INFO_STREAM(this->logger(), GetLoggerPrefix(name()) << ": onHalt");
}

}  // namespace panther_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(panther_manager::UndockRobot, "UndockRobot");
