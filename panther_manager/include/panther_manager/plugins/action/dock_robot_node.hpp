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

#ifndef PANTHER_MANAGER_PLUGINS_ACTION_DOCK_ROBOT_NODE_HPP_
#define PANTHER_MANAGER_PLUGINS_ACTION_DOCK_ROBOT_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opennav_docking_msgs/action/dock_robot.hpp>

namespace panther_manager
{

class DockRobot : public BT::RosActionNode<opennav_docking_msgs::action::DockRobot>
{
  using DockRobotAction = opennav_docking_msgs::action::DockRobot;
  using DockRobotActionResult = DockRobotAction::Result;

public:
  DockRobot(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : RosActionNode<DockRobotAction>(name, conf, params)
  {
  }

  bool setGoal(Goal & goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<bool>(
         "use_dock_id", true, "Determines whether to use the dock's ID or dock pose fields."),
       BT::InputPort<std::string>(
         "dock_id",
         "Specifies the dock's ID or name from the dock database (used if 'use_dock_id' is true)."),
       BT::InputPort<geometry_msgs::msg::PoseStamped>(
         "dock_pose",
         "Specifies the dock's pose (used if 'use_dock_id' is false). Format: "
         "\"x;y;z;roll;pitch;yaw;frame_id\""),
       BT::InputPort<std::string>(
         "dock_type",
         "Defines the type of dock being used when docking via pose. Not needed if only one dock "
         "type is available."),
       BT::InputPort<float>(
         "max_staging_time", 120.0,
         "Maximum time allowed (in seconds) for navigating to the dock's staging pose."),
       BT::InputPort<bool>(
         "navigate_to_staging_pose", true,
         "Specifies whether the robot should autonomously navigate to the staging pose."),

       BT::OutputPort<DockRobotActionResult::_error_code_type>(
         "error_code", "Returns an error code indicating the reason for failure, if any."),
       BT::OutputPort<DockRobotActionResult::_num_retries_type>(
         "num_retries", "Reports the number of retry attempts made during the docking process.")});
  }
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_PLUGINS_ACTION_DOCK_ROBOT_NODE_HPP_
