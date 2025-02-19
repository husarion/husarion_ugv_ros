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

#include <fcntl.h>
#include <sys/wait.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

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

inline void CommandHandler::Execute(
  const std::string & command, const std::chrono::milliseconds & timeout_ms)
{
  timeout_ms_ = timeout_ms;
  state_ = CommandState::RUNNING;
  if (!ExecuteCommandInChildProcess(command)) {
    state_ = CommandState::FAILURE;
  }
}

inline void CommandHandler::Halt()
{
  KillChildProcess();
  state_ = CommandState::FAILURE;
}

inline void CommandHandler::CheckExecution()
{
  while (state_.load() == CommandState::RUNNING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (ReadCommandOutput()) {
      continue;
    }

    int status;
    if (waitpid(m_child_pid_, &status, WNOHANG) == m_child_pid_) {
      close(pipefd_[0]);  // Close read end after reading

      if (WEXITSTATUS(status) != 0) {
        std::lock_guard<std::mutex> lock(error_mtx_);
        error_ = "Command return code: " + std::to_string(WEXITSTATUS(status));
        state_ = CommandState::FAILURE;
      } else {
        state_ = CommandState::SUCCESS;
      }

      break;
    }

    if (TimeoutExceeded(command_time_, timeout_ms_)) {
      KillChildProcess();
      std::lock_guard<std::mutex> lock(error_mtx_);
      error_ = "Timeout exceeded";
      state_ = CommandState::FAILURE;
      break;
    }
  }
}

inline bool CommandHandler::ExecuteCommandInChildProcess(const std::string & command)
{
  // Create a pipe
  if (pipe(pipefd_) == -1) {
    std::lock_guard<std::mutex> lock(error_mtx_);
    error_ = "Failed to create pipe";
    return false;
  }

  // Set the pipe to non-blocking mode
  int flags = fcntl(pipefd_[0], F_GETFL, 0);
  fcntl(pipefd_[0], F_SETFL, flags | O_NONBLOCK);

  // Create a child process that will execute the command
  m_child_pid_ = fork();
  command_time_ = std::chrono::steady_clock::now();

  if (m_child_pid_ == -1) {
    std::lock_guard<std::mutex> lock(error_mtx_);
    error_ = "Failed to fork";
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

  command_checker_thread_ = std::thread(&CommandHandler::CheckExecution, this);

  return true;
}

inline bool CommandHandler::ReadCommandOutput()
{
  char buffer[128];
  ssize_t bytes_read;

  bytes_read = read(pipefd_[0], buffer, sizeof(buffer) - 1);
  if ((bytes_read) > 0) {
    buffer[bytes_read] = '\0';
    std::lock_guard<std::mutex> lock(output_mtx_);
    output_ += buffer;
    return true;
  }

  return false;
}

inline void CommandHandler::KillChildProcess()
{
  if (state_.load() != CommandState::RUNNING) {
    return;
  }
  close(pipefd_[0]);  // Close read end of the pipe
  kill(m_child_pid_, SIGKILL);
  int status;
  waitpid(m_child_pid_, &status, 0);
}

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_ACTION_COMMAND_HANDLER_HPP_
