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

#include <opennav_docking_msgs/action/dock_robot.hpp>

#include "panther_manager/plugins/action/dock_robot_node.hpp"
#include "utils/plugin_test_utils.hpp"

class TestDockRobot : public panther_manager::plugin_test_utils::PluginTestUtils
{
public:
  using Action = opennav_docking_msgs::action::DockRobot;
  using ActionResult = opennav_docking_msgs::action::DockRobot::Result;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
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

    auto handle_cancel = [&, cancel_response](const std::shared_ptr<GoalHandle> /*goal_handle*/)
      -> rclcpp_action::CancelResponse { return cancel_response; };

    auto handle_accepted = [&, success,
                            error_code](const std::shared_ptr<GoalHandle> goal_handle) -> void {
      ActionResult::SharedPtr result = std::make_shared<ActionResult>();
      result->success = success;
      result->error_code = error_code;
      result->num_retries = 0;
      goal_handle->succeed(result);
    };

    CreateAction<Action>("test_dock_action", handle_goal, handle_cancel, handle_accepted);
  }
};

TEST_F(TestDockRobot, GoodLoadingDockRobotPlugin)
{
  std::map<std::string, std::string> params = {
    {"action_name", "test_dock_action"},
    {"use_dock_id", "true"},
    {"dock_id", "test_dock"},
    {"dock_type", "test_dock_type"},
    {"max_staging_time", "5.0"},
    {"navigate_to_staging_pose", "false"},
  };

  RegisterNodeWithParams<panther_manager::DockRobot>("DockRobot");

  ASSERT_NO_THROW({ CreateTree("DockRobot", params); });
}

TEST_F(TestDockRobot, WrongLoadingDockRobotPlugin)
{
  std::map<std::string, std::string> params = {
    {"action_name", ""},         {"use_dock_id", "true"},
    {"dock_id", "test_dock"},    {"dock_type", "test_dock_type"},
    {"max_staging_time", "5.0"}, {"navigate_to_staging_pose", "false"},
  };

  RegisterNodeWithParams<panther_manager::DockRobot>("DockRobot");

  EXPECT_THROW({ CreateTree("WrongDockRobot", params); }, BT::RuntimeError);
}

TEST_F(TestDockRobot, WrongCallDockRobotServerNotInitialized)
{
  std::map<std::string, std::string> params = {
    {"action_name", "test_dock_action"},
    {"use_dock_id", "true"},
    {"dock_id", "test_dock"},
    {"dock_type", "test_dock_type"},
    {"max_staging_time", "5.0"},
    {"navigate_to_staging_pose", "false"},
  };
  RegisterNodeWithParams<panther_manager::DockRobot>("DockRobot");

  CreateTree("DockRobot", params);

  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestDockRobot, WrongCallDockRobotServerWithNoDockID)
{
  CreateActionServer(
    GoalResponse::ACCEPT_AND_EXECUTE, CancelResponse::ACCEPT, true, ActionResult::NONE);

  std::map<std::string, std::string> params = {
    {"action_name", "test_dock_action"},   {"use_dock_id", "true"},
    {"dock_type", "test_dock_type"},       {"max_staging_time", "5.0"},
    {"navigate_to_staging_pose", "false"},
  };

  RegisterNodeWithParams<panther_manager::DockRobot>("DockRobot");
  CreateTree("DockRobot", params);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestDockRobot, CallDockRobotServerWithoutDockID)
{
  CreateActionServer(
    GoalResponse::ACCEPT_AND_EXECUTE, CancelResponse::ACCEPT, true, ActionResult::NONE);

  std::map<std::string, std::string> params = {
    {"action_name", "test_dock_action"},   {"use_dock_id", "false"},
    {"dock_type", "test_dock_type"},       {"max_staging_time", "5.0"},
    {"navigate_to_staging_pose", "false"},
  };

  RegisterNodeWithParams<panther_manager::DockRobot>("DockRobot");
  CreateTree("DockRobot", params);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestDockRobot, CallDockRobotServerWithEmptyDockID)
{
  CreateActionServer(
    GoalResponse::ACCEPT_AND_EXECUTE, CancelResponse::ACCEPT, true, ActionResult::NONE);

  std::map<std::string, std::string> params = {
    {"action_name", "test_dock_action"},
    {"use_dock_id", "true"},
    {"dock_id", ""},
    {"dock_type", "test_dock_type"},
    {"max_staging_time", "5.0"},
    {"navigate_to_staging_pose", "true"},
  };

  RegisterNodeWithParams<panther_manager::DockRobot>("DockRobot");
  CreateTree("DockRobot", params);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestDockRobot, CallDockRobotServerWithEmptyDockType)
{
  CreateActionServer(
    GoalResponse::ACCEPT_AND_EXECUTE, CancelResponse::ACCEPT, true, ActionResult::NONE);

  std::map<std::string, std::string> params = {
    {"action_name", "test_dock_action"},
    {"use_dock_id", "true"},
    {"dock_id", "main_dock"},
    {"dock_type", ""},
    {"max_staging_time", "5.0"},
    {"navigate_to_staging_pose", "true"},
  };

  RegisterNodeWithParams<panther_manager::DockRobot>("DockRobot");
  CreateTree("DockRobot", params);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestDockRobot, CallDockRobotServerWithNavigateToStagingPoseFailure)
{
  CreateActionServer(GoalResponse::REJECT, CancelResponse::ACCEPT, true, ActionResult::NONE);

  std::map<std::string, std::string> params = {
    {"action_name", "test_dock_action"},
    {"use_dock_id", "true"},
    {"dock_id", "main_dock"},
    {"dock_type", "test_dock_type"},
    {"max_staging_time", "5.0"},
    {"navigate_to_staging_pose", "true"},
  };

  RegisterNodeWithParams<panther_manager::DockRobot>("DockRobot");
  CreateTree("DockRobot", params);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestDockRobot, CallDockRobotServerWithNavigateToStagingPoseSuccess)
{
  CreateActionServer(
    GoalResponse::ACCEPT_AND_EXECUTE, CancelResponse::ACCEPT, true, ActionResult::NONE);

  std::map<std::string, std::string> params = {
    {"action_name", "test_dock_action"},
    {"use_dock_id", "true"},
    {"dock_id", "main_dock"},
    {"dock_type", "test_dock_type"},
    {"max_staging_time", "5.0"},
    {"navigate_to_staging_pose", "true"},
  };

  RegisterNodeWithParams<panther_manager::DockRobot>("DockRobot");
  CreateTree("DockRobot", params);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
