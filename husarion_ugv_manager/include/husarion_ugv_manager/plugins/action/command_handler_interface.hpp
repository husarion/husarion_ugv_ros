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

#ifndef HUSARION_UGV_MANAGER_PLUGINS_ACTION_COMMAND_HANDLER_INTERFACE_NODE_HPP_
#define HUSARION_UGV_MANAGER_PLUGINS_ACTION_COMMAND_HANDLER_INTERFACE_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/logger.hpp"

namespace husarion_ugv_manager
{

class CommandHandlerInterface : public BT::StatefulActionNode
{
public:
  explicit CommandHandlerInterface(const std::string & name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    this->logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(name));
  }

  ~CommandHandlerInterface() = default;

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  /**
   * @brief Checks the execution status of the command.
   *
   * @return RUNNING if the command is still being executed, SUCCESS if the command was executed
   * successfully or FAILURE if the command execution failed or timeout was exceeded.
   */
  BT::NodeStatus CheckCommandExecution();

  /**
   * @brief Returns the command to be executed.
   *
   * @return The command to be executed.
   * @throws BT::RuntimeError if the command cannot be retrieved.
   */
  virtual std::string GetCommand() = 0;

  /**
   * @brief Returns the timeout for the command execution
   * in seconds.
   *
   * @return The timeout for the command execution.
   * @throws BT::RuntimeError if the timeout cannot be retrieved.
   */
  virtual float GetTimeout() = 0;

  std::string GetOutput() const { return output_; }

  std::shared_ptr<rclcpp::Logger> logger_;

private:
  /**
   * @brief Orders execution of a command in a child process.
   *
   * @param command The command to be executed.
   * @return true if the command execution was ordered successfully, false otherwise.
   */
  bool ExecuteCommandInChildProcess(const std::string & command);

  /**
   * @brief Read command output and save it to the output_ variable.
   *
   * @return true if it was possible to read the command output, false otherwise.
   */
  bool ReadCommandOutput();

  void KillChildProcess();
  bool TimeoutExceeded();

  int pipefd_[2];
  pid_t m_child_pid_;
  std::string output_;
  std::chrono::milliseconds timeout_ms_;
  std::chrono::time_point<std::chrono::steady_clock> command_time_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_ACTION_COMMAND_HANDLER_INTERFACE_NODE_HPP_
