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

#ifndef HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_PLUGINS_CONDITION_CHECK_STRING_MSG_HPP_
#define HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_PLUGINS_CONDITION_CHECK_STRING_MSG_HPP_

#include <memory>
#include <string>

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

namespace husarion_ugv_manager
{

// FIXME: There is no possibility to set QoS profile. Add it in the future to subscribe e_stop.
class CheckStringMsg : public BT::RosTopicSubNode<std_msgs::msg::String>
{
  using StringMsg = std_msgs::msg::String;

public:
  CheckStringMsg(const std::string & name, const BT::NodeConfig & conf)
  : BT::RosTopicSubNode<StringMsg>(
      name, conf, behavior_tree_utils::CreateRosNodeParamsFromBlackboard(conf))
  {
  }

  CheckStringMsg(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosTopicSubNode<StringMsg>(name, conf, params)
  {
  }

  BT::NodeStatus onTick(const StringMsg::SharedPtr & last_msg) override;

  bool latchLastMessage() const override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<std::string>("data", "Specifies the expected state of the data field.")});
  }
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_PLUGINS_CONDITION_CHECK_STRING_MSG_HPP_
