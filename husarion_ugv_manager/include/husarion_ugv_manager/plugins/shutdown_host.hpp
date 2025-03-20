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
#include <iomanip>
#include <ios>
#include <string>

#include <openssl/hmac.h>

#include "husarion_ugv_manager/behavior_tree_utils.hpp"
#include "husarion_ugv_manager/plugins/command_handler.hpp"

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

class ShutdownHostInterface
{
public:
  ShutdownHostInterface(const std::size_t hash) : hash_(hash) {}
  ~ShutdownHostInterface() {}

  virtual void Call() = 0;
  virtual void Halt() = 0;
  virtual std::string GetIp() const = 0;
  virtual std::string GetError() const = 0;
  virtual std::string GetOutput() const = 0;
  virtual ShutdownHostState GetState() const = 0;

  bool operator==(const ShutdownHostInterface & other) const { return hash_ == other.hash_; }
  bool operator!=(const ShutdownHostInterface & other) const { return hash_ != other.hash_; }
  bool operator<(const ShutdownHostInterface & other) const { return hash_ < other.hash_; }

private:
  const std::size_t hash_;
};

class ShutdownHost : public ShutdownHostInterface
{
public:
  ShutdownHost()
  : ShutdownHostInterface(std::hash<std::string>{}("")),
    ip_(""),
    port_("3003"),
    secret_(""),
    timeout_ms_(5000)
  {
  }
  ShutdownHost(
    const std::string ip, const std::string & port = "3003", const std::string secret = "husarion",
    const float timeout = 5.0)
  : ShutdownHostInterface(std::hash<std::string>{}(ip + port)),
    ip_(ip),
    port_(port),
    secret_(secret),
    timeout_ms_(static_cast<long long>(timeout * 1000)),
    state_(ShutdownHostState::IDLE)
  {
    command_handler_ = std::make_shared<CommandHandler>();
  }

  ~ShutdownHost() = default;

  void Call() override
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
        if (CommandRunning()) {
          break;
        }

        if (command_handler_->GetState() == CommandState::FAILURE) {
          state_ = ShutdownHostState::FAILURE;
          failure_reason_ = command_handler_->GetError();
          break;
        }

        state_ = ShutdownHostState::RESPONSE_RECEIVED;
        break;

      case ShutdownHostState::RESPONSE_RECEIVED:
        if (CheckServerResponse()) {
          state_ = ShutdownHostState::PINGING;
          break;
        }
        state_ = ShutdownHostState::FAILURE;
        break;

      case ShutdownHostState::PINGING:
        if (!IsAvailable()) {
          state_ = ShutdownHostState::SUCCESS;
          break;
        }
        if (TimeoutExceeded(request_time_, timeout_ms_)) {
          state_ = ShutdownHostState::FAILURE;
          failure_reason_ = "Timeout waiting for host to shutdown";
        }
        break;

      default:
        break;
    }
  }

  void Halt() override { command_handler_->Halt(); }

  std::string GetIp() const override { return ip_; }
  std::string GetError() const override { return failure_reason_; }
  std::string GetOutput() const override { return command_handler_->GetOutput(); }
  ShutdownHostState GetState() const override { return state_; }

protected:
  bool IsAvailable() const
  {
    return system(("ping -c 1 -w 1 " + ip_ + " > /dev/null").c_str()) == 0;
  }

  /**
   * @brief Get the current time since epoch in seconds.
   *
   * @return std::int64_t The current time since epoch in seconds.
   */
  std::int64_t GetTimeSinceEpoch()
  {
    return std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
  }

private:
  void RequestShutdown()
  {
    request_time_ = std::chrono::steady_clock::now();
    const auto time_now_str = std::to_string(GetTimeSinceEpoch());
    std::string string_to_sign = "/shutdown|" + time_now_str;

    unsigned char * hmac_result;
    unsigned int len = 32;
    hmac_result = HMAC(
      EVP_sha256(), secret_.c_str(), secret_.length(),
      reinterpret_cast<const unsigned char *>(string_to_sign.c_str()), string_to_sign.length(),
      NULL, NULL);

    std::stringstream ss;
    for (unsigned int i = 0; i < len; i++) {
      ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hmac_result[i]);
    }
    std::string sig = ss.str();

    const std::string command = "curl -s -w '%{errormsg}\\n%{http_code}' 'http://" + ip_ + ":" +
                                port_ + "/shutdown?ts=" + time_now_str + "&sig=" + sig + "'";

    command_handler_->Execute(command, timeout_ms_);
    if (command_handler_->GetState() == CommandState::FAILURE) {
      throw std::runtime_error("Failed to execute command");
    }
  }

  bool CommandRunning()
  {
    return command_handler_->GetState() == CommandState::RUNNING ? true : false;
  }

  bool CheckServerResponse()
  {
    const auto output = this->command_handler_->GetOutput();
    // Output may have multiple lines. We are interested in the last one.
    const auto http_return_code = output.substr(output.find_last_of('\n') + 1);
    if (http_return_code != "200") {
      failure_reason_ = "Failed to shutdown remote host. Server return code: " + http_return_code;
      return false;
    }

    return true;
  }

  const std::string ip_;
  const std::string port_;
  const std::string secret_;
  const std::chrono::milliseconds timeout_ms_;
  ShutdownHostState state_;

  std::chrono::time_point<std::chrono::steady_clock> request_time_;

  std::string failure_reason_;

  std::shared_ptr<CommandHandler> command_handler_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_SHUTDOWN_HOST_HPP_
