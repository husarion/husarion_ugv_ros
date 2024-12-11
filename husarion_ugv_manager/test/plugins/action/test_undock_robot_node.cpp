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

#include <cstdint>
#include <map>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <opennav_docking_msgs/action/undock_robot.hpp>

#include "panther_manager/plugins/action/undock_robot_node.hpp"
#include "utils/plugin_test_utils.hpp"

class TestUndockRobot : public panther_manager::plugin_test_utils::PluginTestUtils
{
public:
  using Action = opennav_docking_msgs::action::UndockRobot;
  using ActionResult = opennav_docking_msgs::action::UndockRobot::Result;
  using GoalHandleAction = rclcpp_action::ServerGoalHandle<Action>;
  using GoalResponse = rclcpp_action::GoalResponse;
  using CancelResponse = rclcpp_action::CancelResponse;

  void CreateActionServer(
    GoalResponse goal_response, CancelResponse cancel_response, bool success,
    std::uint16_t error_code)
  {
    auto handle_goal =
      [&, goal_response](
        const rclcpp_action::GoalUUID & /*uuid*/,
        std::shared_ptr<const Action::Goal> /*goal*/) -> rclcpp_action::GoalResponse {
      return goal_response;
    };

    auto handle_cancel =
      [&, cancel_response](
        const std::shared_ptr<GoalHandleAction> /*goal_handle*/) -> rclcpp_action::CancelResponse {
      return cancel_response;
    };

    auto handle_accepted =
      [&, success, error_code](const std::shared_ptr<GoalHandleAction> goal_handle) -> void {
      ActionResult::SharedPtr result = std::make_shared<ActionResult>();
      result->success = success;
      result->error_code = error_code;
      goal_handle->succeed(result);
    };

    CreateAction<Action>("test_undock_action", handle_goal, handle_cancel, handle_accepted);
  }
};

TEST_F(TestUndockRobot, GoodLoadingUndockRobotPlugin)
{
  std::map<std::string, std::string> params = {
    {"action_name", "test_undock_action"},
    {"dock_type", "test_dock_type"},
    {"max_undocking_time", "5.0"},
  };

  RegisterNodeWithParams<panther_manager::UndockRobot>("UndockRobot");

  ASSERT_NO_THROW({ CreateTree("UndockRobot", params); });
}

TEST_F(TestUndockRobot, WrongLoadingUndockRobotPlugin)
{
  std::map<std::string, std::string> params = {
    {"action_name", ""},
    {"dock_type", "test_dock_type"},
    {"max_undocking_time", "5.0"},
  };

  RegisterNodeWithParams<panther_manager::UndockRobot>("UndockRobot");

  EXPECT_THROW({ CreateTree("WrongUndockRobot", params); }, BT::RuntimeError);
}

TEST_F(TestUndockRobot, WrongCallUndockRobotServerNotInitialized)
{
  std::map<std::string, std::string> params = {
    {"action_name", "test_undock_action"},
    {"dock_type", "test_dock_type"},
    {"max_undocking_time", "5.0"},
  };

  RegisterNodeWithParams<panther_manager::UndockRobot>("UndockRobot");

  CreateTree("UndockRobot", params);

  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestUndockRobot, WrongCallUndockRobotServerWithNoDockType)
{
  CreateActionServer(
    GoalResponse::ACCEPT_AND_EXECUTE, CancelResponse::ACCEPT, true, ActionResult::NONE);

  std::map<std::string, std::string> params = {
    {"action_name", "test_undock_action"},
    {"dock_type", ""},
    {"max_undocking_time", "5.0"},
  };

  RegisterNodeWithParams<panther_manager::UndockRobot>("UndockRobot");
  CreateTree("UndockRobot", params);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(1000));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestUndockRobot, CallUndockRobotServerFailure)
{
  CreateActionServer(GoalResponse::REJECT, CancelResponse::ACCEPT, true, ActionResult::NONE);

  std::map<std::string, std::string> params = {
    {"action_name", "test_undock_action"},
    {"dock_type", "test_dock_type"},
    {"max_undocking_time", "5.0"},
  };

  RegisterNodeWithParams<panther_manager::UndockRobot>("UndockRobot");
  CreateTree("UndockRobot", params);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(1000));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestUndockRobot, CallUndockRobotServerSuccess)
{
  CreateActionServer(
    GoalResponse::ACCEPT_AND_EXECUTE, CancelResponse::ACCEPT, true, ActionResult::NONE);

  std::map<std::string, std::string> params = {
    {"action_name", "test_undock_action"},
    {"dock_type", "test_dock_type"},
    {"max_undocking_time", "5.0"},
  };

  RegisterNodeWithParams<panther_manager::UndockRobot>("UndockRobot");
  CreateTree("UndockRobot", params);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(1000));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
