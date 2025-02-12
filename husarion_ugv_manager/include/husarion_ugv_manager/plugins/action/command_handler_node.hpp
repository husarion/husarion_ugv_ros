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

#ifndef HUSARION_UGV_MANAGER_PLUGINS_ACTION_EXECUTE_COMMAND_NODE_HPP_
#define HUSARION_UGV_MANAGER_PLUGINS_ACTION_EXECUTE_COMMAND_NODE_HPP_

#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/logger.hpp"

#include <iostream>

namespace husarion_ugv_manager
{

class CommandHandler : public BT::StatefulActionNode
{
public:
  explicit CommandHandler(const std::string & name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    this->logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(name));
  }

  ~CommandHandler() = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("command", "Command to execute."),
      BT::InputPort<float>("timeout", "Command time out in seconds."),
    };
  }

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  /**
   * @brief Executes a given command.
   *
   * @param command The command to be executed as a string.
   * @return int The result of the command execution. Typically, 0 indicates success, while non-zero
   * values indicate errors.
   */
  // int ExecuteCommand(const std::string & command);

  bool TimeoutExceeded();

  std::shared_ptr<rclcpp::Logger> logger_;
  // std::string result_;
  // FILE * pipe_;

  std::chrono::milliseconds timeout_ms_;
  std::chrono::time_point<std::chrono::steady_clock> command_time_;
  // std::future<int> command_future_;
  // std::thread command_therad_;
  // std::atomic<bool> command_running_;
  // std::atomic<bool> command_timeout_;

  pid_t m_child_pid_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_ACTION_EXECUTE_COMMAND_NODE_HPP_
