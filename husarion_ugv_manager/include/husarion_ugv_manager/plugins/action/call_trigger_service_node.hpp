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

#ifndef HUSARION_UGV_MANAGER_PLUGINS_ACTION_CALL_TRIGGER_SERVICE_NODE_HPP_
#define HUSARION_UGV_MANAGER_PLUGINS_ACTION_CALL_TRIGGER_SERVICE_NODE_HPP_

#include <string>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"

namespace husarion_ugv_manager
{

class CallTriggerService : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  CallTriggerService(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  virtual bool setRequest(typename Request::SharedPtr & /*request*/) override;
  virtual BT::NodeStatus onResponseReceived(const typename Response::SharedPtr & response) override;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_ACTION_CALL_TRIGGER_SERVICE_NODE_HPP_
