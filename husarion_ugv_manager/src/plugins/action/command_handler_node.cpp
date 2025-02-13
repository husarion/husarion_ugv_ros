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

#include <fcntl.h>
#include <sys/wait.h>

#include <chrono>
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

  if (ExecuteCommandInChildProcess(command)) {
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CommandHandler::onRunning()
{
  if (ReadAndLogCommandOutput()) {
    return BT::NodeStatus::RUNNING;
  }

  int status;
  if (waitpid(m_child_pid_, &status, WNOHANG) == m_child_pid_) {
    close(pipefd_[0]);  // Close read end after reading

    if (WEXITSTATUS(status) != 0) {
      RCLCPP_ERROR_STREAM(
        *this->logger_, GetLoggerPrefix(name())
                          << "Command failed with status: " << WEXITSTATUS(status));
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  if (TimeoutExceeded()) {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Command timed out");
    KillChildProcess();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void CommandHandler::onHalted() { KillChildProcess(); }

bool CommandHandler::ExecuteCommandInChildProcess(const std::string & command)
{
  // Create a pipe
  if (pipe(pipefd_) == -1) {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Failed to create pipe");
    return false;
  }

  // Set the pipe to non-blocking mode
  int flags = fcntl(pipefd_[0], F_GETFL, 0);
  fcntl(pipefd_[0], F_SETFL, flags | O_NONBLOCK);

  // Create a child process that will execute the command
  RCLCPP_INFO_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Executing command: " << command);
  m_child_pid_ = fork();
  command_time_ = std::chrono::steady_clock::now();

  if (m_child_pid_ == -1) {
    return false;
  }

  if (m_child_pid_ == 0) {
    close(pipefd_[0]);                // Close unused read end
    dup2(pipefd_[1], STDOUT_FILENO);  // Redirect stdout to pipe
    dup2(pipefd_[1], STDERR_FILENO);  // Redirect stderr to pipe
    close(pipefd_[1]);                // Close write end after redirecting

    execl("/bin/bash", "bash", "-c", command.c_str(), nullptr);
    exit(EXIT_FAILURE);
  }

  close(pipefd_[1]);  // Close unused write end

  return true;
}

bool CommandHandler::ReadAndLogCommandOutput()
{
  char buffer[128];
  ssize_t bytes_read;

  bytes_read = read(pipefd_[0], buffer, sizeof(buffer) - 1);
  if ((bytes_read) > 0) {
    buffer[bytes_read] = '\0';
    RCLCPP_INFO_STREAM(*this->logger_, GetLoggerPrefix(name()) << buffer);
    return true;
  }

  return false;
}

void CommandHandler::KillChildProcess()
{
  close(pipefd_[0]);  // Close read end of the pipe
  kill(m_child_pid_, SIGKILL);
  int status;
  waitpid(m_child_pid_, &status, 0);
}

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
