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

#ifndef HUSARION_UGV_MANAGER_PLUGINS_SHUTDOWN_HOST_HPP_
#define HUSARION_UGV_MANAGER_PLUGINS_SHUTDOWN_HOST_HPP_

#include <chrono>
#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libssh/libsshpp.hpp"

namespace husarion_ugv_manager
{

enum class ShutdownHostState {
  IDLE = 0,
  COMMAND_EXECUTED,
  RESPONSE_RECEIVED,
  PINGING,
  SKIPPED,
  SUCCESS,
  FAILURE,
};

class ShutdownHost
{
public:
  ShutdownHost()
  : ip_(""),
    user_(""),
    port_(22),
    command_(""),
    timeout_ms_(5000),
    ping_for_success_(true),
    hash_(std::hash<std::string>{}(""))
  {
  }
  ShutdownHost(
    const std::string ip, const std::string user, const int port = 22,
    const std::string command = "sudo shutdown now", const float timeout = 5.0,
    const bool ping_for_success = true)
  : ip_(ip),
    user_(user),
    port_(port),
    command_(command),
    timeout_ms_(static_cast<long long>(timeout * 1000)),
    ping_for_success_(ping_for_success),
    hash_(std::hash<std::string>{}(ip + user + std::to_string(port))),
    state_(ShutdownHostState::IDLE)
  {
  }

  ~ShutdownHost() {}

  void Call()
  {
    switch (state_) {
      case ShutdownHostState::IDLE:
        if (!IsAvailable()) {
          state_ = ShutdownHostState::SKIPPED;
          break;
        }

        try {
          RequestShutdown();
        } catch (const std::runtime_error & err) {
          state_ = ShutdownHostState::FAILURE;
          failure_reason_ = err.what();
          break;
        }
        state_ = ShutdownHostState::COMMAND_EXECUTED;
        break;

      case ShutdownHostState::COMMAND_EXECUTED:
        try {
          if (UpdateResponse()) {
            break;
          }
        } catch (const std::runtime_error & err) {
          state_ = ShutdownHostState::FAILURE;
          failure_reason_ = err.what();
          break;
        }
        state_ = ShutdownHostState::RESPONSE_RECEIVED;
        break;

      case ShutdownHostState::RESPONSE_RECEIVED:
        state_ = ShutdownHostState::PINGING;
        break;

      case ShutdownHostState::PINGING:
        if (ping_for_success_ ? !IsAvailable() : true) {
          state_ = ShutdownHostState::SUCCESS;
          break;
        }
        if (TimeoutExceeded()) {
          state_ = ShutdownHostState::FAILURE;
          failure_reason_ = "Timeout exceeded";
        }
        break;

      default:
        break;
    }
  }

  bool IsAvailable() const
  {
    return system(("ping -c 1 -w 1 " + ip_ + " > /dev/null").c_str()) == 0;
  }

  void CloseConnection()
  {
    if (ssh_channel_is_closed(channel_)) {
      return;
    }

    ssh_channel_send_eof(channel_);
    ssh_channel_close(channel_);
    ssh_channel_free(channel_);
    ssh_disconnect(session_);
    ssh_free(session_);
  }

  int GetPort() const { return port_; }

  std::string GetIp() const { return ip_; }

  std::string GetUser() const { return user_; }

  std::string GetCommand() const { return command_; }

  std::string GetError() const { return failure_reason_; }

  std::string GetResponse() const { return output_; }

  ShutdownHostState GetState() const { return state_; }

  bool operator==(const ShutdownHost & other) const { return hash_ == other.hash_; }

  bool operator!=(const ShutdownHost & other) const { return hash_ != other.hash_; }

  bool operator<(const ShutdownHost & other) const { return hash_ < other.hash_; }

private:
  const std::string ip_;
  const std::string user_;
  const int port_;
  const std::string command_;
  const std::chrono::milliseconds timeout_ms_;
  const bool ping_for_success_;
  const std::size_t hash_;
  ShutdownHostState state_;

  char buffer_[1024];
  const int verbosity_ = SSH_LOG_NOLOG;
  int nbytes_;
  std::string output_;
  std::string failure_reason_;
  std::chrono::time_point<std::chrono::steady_clock> command_time_;

  ssh_session session_;
  ssh_channel channel_;

  void RequestShutdown()
  {
    SshExecuteCommand(command_);
    command_time_ = std::chrono::steady_clock::now();
  }

  bool UpdateResponse()
  {
    if (!IsAvailable()) {
      CloseConnection();
      throw std::runtime_error("Lost connection");
    }

    if (!ssh_channel_is_open(channel_)) {
      throw std::runtime_error("Channel closed");
    }

    if (TimeoutExceeded()) {
      CloseConnection();
      throw std::runtime_error("Timeout exceeded");
    }

    if ((nbytes_ = ssh_channel_read_nonblocking(channel_, buffer_, sizeof(buffer_), 0)) >= 0) {
      output_.append(buffer_, nbytes_);
      return true;
    }
    CloseConnection();
    return false;
  }

  bool TimeoutExceeded()
  {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - command_time_);

    return elapsed > timeout_ms_ && IsAvailable();
  }

  void SshExecuteCommand(const std::string & command)
  {
    session_ = ssh_new();
    if (session_ == NULL) {
      throw std::runtime_error("Failed to open session");
    };

    ssh_options_set(session_, SSH_OPTIONS_HOST, ip_.c_str());
    ssh_options_set(session_, SSH_OPTIONS_USER, user_.c_str());
    ssh_options_set(session_, SSH_OPTIONS_PORT, &port_);
    ssh_options_set(session_, SSH_OPTIONS_LOG_VERBOSITY, &verbosity_);

    if (ssh_connect(session_) != SSH_OK) {
      std::string err = ssh_get_error(session_);
      ssh_free(session_);
      throw std::runtime_error("Error connecting to host: " + err);
    }

    if (ssh_userauth_publickey_auto(session_, NULL, NULL) != SSH_AUTH_SUCCESS) {
      std::string err = ssh_get_error(session_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Error authenticating with public key: " + err);
    }

    channel_ = ssh_channel_new(session_);
    if (channel_ == NULL) {
      std::string err = ssh_get_error(session_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Failed to create ssh channel: " + err);
    }

    if (ssh_channel_open_session(channel_) != SSH_OK) {
      std::string err = ssh_get_error(session_);
      ssh_channel_free(channel_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Failed to open ssh channel: " + err);
    }

    if (ssh_channel_request_exec(channel_, command.c_str()) != SSH_OK) {
      std::string err = ssh_get_error(session_);
      ssh_channel_close(channel_);
      ssh_channel_free(channel_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Failed to execute ssh command: " + err);
    }
  }
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_SHUTDOWN_HOST_HPP_
