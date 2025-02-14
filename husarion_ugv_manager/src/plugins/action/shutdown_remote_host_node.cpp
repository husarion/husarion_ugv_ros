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

#include "husarion_ugv_manager/plugins/action/shutdown_remote_host_node.hpp"

#include <chrono>
#include <iomanip>
#include <ios>
#include <string>

#include <openssl/hmac.h>

#include "behaviortree_cpp/exceptions.h"

namespace husarion_ugv_manager
{

std::string ShutdownRemoteHost::GetCommand()
{
  std::string command;

  std::string server_ip;
  if (!this->getInput<std::string>("server_ip", server_ip)) {
    throw BT::RuntimeError("Failed to get input [server_ip]");
  }

  std::string server_port;
  if (!this->getInput<std::string>("server_port", server_port)) {
    throw BT::RuntimeError("Failed to get input [server_port]");
  }

  std::string secret;
  if (!this->getInput<std::string>("secret", secret)) {
    throw BT::RuntimeError("Failed to get input [secret]");
  }

  std::string time_now =
    std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
  std::string string_to_sign = "/shutdown|" + time_now;

  unsigned char * hmac_result;
  unsigned int len = 32;
  hmac_result = HMAC(
    EVP_sha256(), secret.c_str(), secret.length(),
    reinterpret_cast<const unsigned char *>(string_to_sign.c_str()), string_to_sign.length(), NULL,
    NULL);

  std::stringstream ss;
  for (unsigned int i = 0; i < len; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hmac_result[i]);
  }
  std::string sig = ss.str();

  command = "curl 'http://" + server_ip + ":" + server_port + "/shutdown?ts=" + time_now +
            "&sig=" + sig + "'";

  std::cout << "Command: " << command << std::endl;
  return command;
}

float ShutdownRemoteHost::GetTimeout()
{
  float timeout;
  if (!this->getInput<float>("timeout", timeout)) {
    throw BT::RuntimeError("Failed to get input [timeout]");
  }
  return timeout;
}

}  // namespace husarion_ugv_manager

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost");
}
