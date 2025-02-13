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

#include <string>

#include "behaviortree_cpp/exceptions.h"

namespace husarion_ugv_manager
{

std::string CommandHandler::GetCommand()
{
  std::string command;
  if (!this->getInput<std::string>("command", command)) {
    throw BT::RuntimeError("Failed to get input [command]");
  }
  return command;
}

float CommandHandler::GetTimeout()
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
  factory.registerNodeType<husarion_ugv_manager::CommandHandler>("CommandHandler");
}
