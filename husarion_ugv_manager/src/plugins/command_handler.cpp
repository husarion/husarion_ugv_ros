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

#include "husarion_ugv_manager/plugins/command_handler.hpp"

#include <fcntl.h>
#include <sys/wait.h>

#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

namespace husarion_ugv_manager
{

void CommandHandler::Execute(
  const std::string & command, const std::chrono::milliseconds & timeout_ms)
{
  timeout_ms_ = timeout_ms;
  state_ = CommandState::RUNNING;
  if (!ExecuteCommandInChildProcess(command)) {
    state_ = CommandState::FAILURE;
  }
}

void CommandHandler::Halt()
{
  KillChildProcess();
  state_ = CommandState::FAILURE;
}

void CommandHandler::CheckExecution()
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

bool CommandHandler::ExecuteCommandInChildProcess(const std::string & command)
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

bool CommandHandler::ReadCommandOutput()
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

void CommandHandler::KillChildProcess()
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
