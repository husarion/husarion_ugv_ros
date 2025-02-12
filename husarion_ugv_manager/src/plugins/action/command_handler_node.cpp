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

#include "husarion_ugv_manager/plugins/action/command_handler_node.hpp"

#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <string>

#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/logger.hpp"

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

namespace husarion_ugv_manager
{

BT::NodeStatus CommandHandler::onStart()
{
  std::string command;
  if (!getInput<std::string>("command", command)) {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Failed to get input [command]");
    return BT::NodeStatus::FAILURE;
  }

  float timeout;
  if (!getInput<float>("timeout", timeout)) {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Failed to get input [timeout]");
    return BT::NodeStatus::FAILURE;
  }

  timeout_ms_ = std::chrono::milliseconds(static_cast<int>(timeout * 1000));

  // Create a child process that will execute the command
  m_child_pid_ = fork();
  command_time_ = std::chrono::steady_clock::now();

  if (m_child_pid_ == -1) {
    return BT::NodeStatus::FAILURE;
  }

  if (m_child_pid_ == 0) {
    RCLCPP_INFO_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Executing command: " << command);
    execl("/bin/bash", "bash", "-c", command.c_str(), nullptr);
    exit(EXIT_FAILURE);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CommandHandler::onRunning()
{
  int status;
  if (waitpid(m_child_pid_, &status, WNOHANG) == m_child_pid_) {
    if (WEXITSTATUS(status) != 0) {
      RCLCPP_ERROR_STREAM(
        *this->logger_, GetLoggerPrefix(name())
                          << "Command failed with status: " << WEXITSTATUS(status));
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  if (TimeoutExceeded()) {
    kill(m_child_pid_, SIGKILL);
    waitpid(m_child_pid_, &status, 0);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void CommandHandler::onHalted()
{
  kill(m_child_pid_, SIGKILL);
  int status;
  waitpid(m_child_pid_, &status, 0);
}

// int CommandHandler::ExecuteCommand(const std::string & command)
// {
//   RCLCPP_INFO_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Executing command: " <<
//   command); command_running_ = true; std::cout << "Executing command: " << command << std::endl;
//   int result = std::system(command.c_str());
//   std::cout << "Command result: " << result << std::endl;
//   command_running_ = false;

//   return result;
// }

bool CommandHandler::TimeoutExceeded()
{
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - command_time_);
  return elapsed > timeout_ms_;
}

}  // namespace husarion_ugv_manager

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<husarion_ugv_manager::CommandHandler>("CommandHandler");
}
