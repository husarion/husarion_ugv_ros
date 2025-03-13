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

#include "husarion_ugv_manager/plugins/action/call_set_led_animation_service_node.hpp"

#include <string>

#include "behaviortree_cpp/basic_types.h"

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

namespace husarion_ugv_manager
{

bool CallSetLedAnimationService::setRequest(typename Request::SharedPtr & request)
{
  unsigned animation_id;
  if (!getInput<unsigned>("id", animation_id)) {
    RCLCPP_ERROR_STREAM(this->logger(), GetLoggerPrefix(name()) << "Failed to get input [id]");
    return false;
  }

  request->animation.id = static_cast<uint16_t>(animation_id);

  if (!getInput<std::string>("param", request->animation.param)) {
    RCLCPP_ERROR_STREAM(this->logger(), GetLoggerPrefix(name()) << "Failed to get input [param]");
    return false;
  }

  if (!getInput<bool>("repeating", request->repeating)) {
    RCLCPP_ERROR_STREAM(
      this->logger(), GetLoggerPrefix(name()) << "Failed to get input [repeating]");
    return false;
  }

  return true;
}

BT::NodeStatus CallSetLedAnimationService::onResponseReceived(
  const typename Response::SharedPtr & response)
{
  if (!response->success) {
    RCLCPP_ERROR_STREAM(
      this->logger(), GetLoggerPrefix(name()) << "Failed to call " << this->service_name_
                                              << "service, message: " << response->message);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_DEBUG_STREAM(
    this->logger(), GetLoggerPrefix(name()) << "Successfully called " << this->service_name_
                                            << " service, message: " << response->message);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace husarion_ugv_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(husarion_ugv_manager::CallSetLedAnimationService, "CallSetLedAnimationService");
