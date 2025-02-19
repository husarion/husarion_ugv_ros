// Copyright 2025 Husarion sp. z o.o.
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

#include "husarion_ugv_manager/plugins/action/execute_command_node.hpp"

#include <string>

#include "behaviortree_cpp/exceptions.h"

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

namespace husarion_ugv_manager
{

BT::NodeStatus ExecuteCommand::onStart()
{
  std::string command;
  if (!this->getInput<std::string>("command", command)) {
    RCLCPP_ERROR_STREAM(*logger_, GetLoggerPrefix(name()) << "Failed to get input [command]");
    return BT::NodeStatus::FAILURE;
  }

  float timeout;
  if (!this->getInput<float>("timeout", timeout)) {
    RCLCPP_ERROR_STREAM(*logger_, GetLoggerPrefix(name()) << "Failed to get input [timeout]");
    return BT::NodeStatus::FAILURE;
  }

  const auto timeout_ms = std::chrono::milliseconds(static_cast<long long>(timeout * 1000));
  command_handler_->Execute(command, timeout_ms);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteCommand::onRunning()
{
  if (command_handler_->GetState() == CommandState::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }

  if (command_handler_->GetState() == CommandState::SUCCESS) {
    RCLCPP_INFO_STREAM(
      *logger_, GetLoggerPrefix(name()) << "Command output: " << command_handler_->GetOutput());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR_STREAM(
    *logger_, GetLoggerPrefix(name()) << "Command failed: " << command_handler_->GetError());
  RCLCPP_INFO_STREAM(
    *logger_, GetLoggerPrefix(name()) << "Command output: " << command_handler_->GetOutput());
  return BT::NodeStatus::FAILURE;
}

void ExecuteCommand::onHalted() { command_handler_->Halt(); }

}  // namespace husarion_ugv_manager

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<husarion_ugv_manager::ExecuteCommand>("ExecuteCommand");
}
