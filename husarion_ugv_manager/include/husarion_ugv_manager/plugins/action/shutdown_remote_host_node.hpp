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

#ifndef HUSARION_UGV_MANAGER_PLUGINS_ACTION_SHUTDOWN_REMOTE_HOST_NODE_HPP_
#define HUSARION_UGV_MANAGER_PLUGINS_ACTION_SHUTDOWN_REMOTE_HOST_NODE_HPP_

#include <string>

#include "behaviortree_cpp/basic_types.h"

#include "husarion_ugv_manager/plugins/action/command_handler_interface.hpp"

namespace husarion_ugv_manager
{

class ShutdownRemoteHost : public CommandHandlerInterface
{
public:
  explicit ShutdownRemoteHost(const std::string & name, const BT::NodeConfig & conf)
  : CommandHandlerInterface(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("server_ip", "IP address of the server to shutdown."),
      BT::InputPort<std::string>("server_port", "Port of the server to shutdown."),
      BT::InputPort<std::string>("secret", "Secret key for HMAC."),
      BT::InputPort<float>("timeout", "Command timeout in seconds."),
    };
  }

protected:
  std::string GetCommand() override;
  float GetTimeout() override;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_ACTION_SHUTDOWN_REMOTE_HOST_NODE_HPP_
