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

#ifndef HUSARION_UGV_MANAGER_PLUGINS_ACTION_COMMAND_HANDLER_HPP_
#define HUSARION_UGV_MANAGER_PLUGINS_ACTION_COMMAND_HANDLER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace husarion_ugv_manager
{

enum class CommandState {
  IDLE = 0,
  RUNNING,
  SUCCESS,
  FAILURE,
};

class CommandHandler
{
public:
  explicit CommandHandler() {}

  ~CommandHandler()
  {
    KillChildProcess();
    if (command_checker_thread_.joinable()) {
      command_checker_thread_.join();
    }
  };

  /**
   * @brief Executes a command in a child process.
   *
   * @param command The command to be executed.
   * @param timeout Timeout for the command.
   */
  void Execute(const std::string & command, const std::chrono::milliseconds & timeout);

  void Halt();

  CommandState GetState() { return state_.load(); }

  std::string GetOutput()
  {
    std::lock_guard<std::mutex> lock(output_mtx_);
    return output_;
  }

  std::string GetError()
  {
    std::lock_guard<std::mutex> lock(error_mtx_);
    return error_;
  }

private:
  /**
   * @brief Checks the execution status of the command and updates the state.
   */
  void CheckExecution();

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

  int pipefd_[2];
  pid_t m_child_pid_;
  std::chrono::milliseconds timeout_ms_;
  std::chrono::time_point<std::chrono::steady_clock> command_time_;

  std::atomic<CommandState> state_{CommandState::IDLE};
  std::string output_;
  std::mutex output_mtx_;
  std::string error_;
  std::mutex error_mtx_;
  std::thread command_checker_thread_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_ACTION_COMMAND_HANDLER_HPP_
