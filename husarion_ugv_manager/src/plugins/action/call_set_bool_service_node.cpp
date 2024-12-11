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

#include "husarion_ugv_manager/plugins/action/call_set_bool_service_node.hpp"

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

namespace husarion_ugv_manager
{

bool CallSetBoolService::setRequest(typename Request::SharedPtr & request)
{
  if (!getInput<bool>("data", request->data)) {
    RCLCPP_ERROR_STREAM(this->logger(), GetLoggerPrefix(name()) << "Failed to get input [data]");
    return false;
  }
  return true;
}

BT::NodeStatus CallSetBoolService::onResponseReceived(const typename Response::SharedPtr & response)
{
  if (!response->success) {
    RCLCPP_ERROR_STREAM(
      this->logger(), GetLoggerPrefix(name()) << "Failed to call " << this->service_name_
                                              << " service, message: " << response->message);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_DEBUG_STREAM(
    this->logger(), GetLoggerPrefix(name()) << "Successfully called " << this->service_name_
                                            << " service, message: " << response->message);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace husarion_ugv_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(husarion_ugv_manager::CallSetBoolService, "CallSetBoolService");
