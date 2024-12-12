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

#include "husarion_ugv_manager/plugins/condition/check_bool_msg.hpp"

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

namespace husarion_ugv_manager
{

BT::NodeStatus CheckBoolMsg::onTick(const BoolMsg::SharedPtr & last_msg)
{
  bool expected_data;
  getInput<bool>("data", expected_data);

  return (last_msg && last_msg->data == expected_data) ? BT::NodeStatus::SUCCESS
                                                       : BT::NodeStatus::FAILURE;
}

}  // namespace husarion_ugv_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(husarion_ugv_manager::CheckBoolMsg, "CheckBoolMsg");
