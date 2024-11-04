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

#ifndef PANTHER_MANAGER_PLUGINS_ACTION_UNDOCK_ROBOT_NODE_HPP_
#define PANTHER_MANAGER_PLUGINS_ACTION_UNDOCK_ROBOT_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opennav_docking_msgs/action/undock_robot.hpp>

namespace panther_manager
{

class UndockRobot : public BT::RosActionNode<opennav_docking_msgs::action::UndockRobot>
{
  using UndockRobotAction = opennav_docking_msgs::action::UndockRobot;
  using UndockRobotActionResult = UndockRobotAction::Result;

public:
  UndockRobot(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : RosActionNode<UndockRobotAction>(name, conf, params)
  {
  }

  bool setGoal(Goal & goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<std::string>(
         "dock_type", "Specifies the dock plugin type to use for undocking."),
       BT::InputPort<float>(
         "max_undocking_time", 30.0,
         "Maximum allowable time (in seconds) to undock and return to the staging pose."),

       BT::OutputPort<UndockRobotActionResult::_error_code_type>(
         "error_code", "Returns an error code if the undocking process fails.")});
  }
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_PLUGINS_ACTION_UNDOCK_ROBOT_NODE_HPP_
