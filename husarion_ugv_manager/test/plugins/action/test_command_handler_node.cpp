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

#include <cstdint>
#include <string>

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "husarion_ugv_manager/plugins/action/command_handler_node.hpp"
#include "utils/plugin_test_utils.hpp"

typedef husarion_ugv_manager::plugin_test_utils::PluginTestUtils TestCommandHandler;

TEST_F(TestCommandHandler, RegisterAndCreateTree)
{
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::CommandHandler>("CommandHandler"));
  ASSERT_NO_THROW(CreateTree("CommandHandler", {}));
}

TEST_F(TestCommandHandler, CallSimpleCommand)
{
  std::map<std::string, std::string> bb_ports = {
    {"command", "echo 'Test command'"}, {"timeout", "1.0"}};
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::CommandHandler>("CommandHandler"));
  ASSERT_NO_THROW(CreateTree("CommandHandler", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCommandHandler, InvalidCommand)
{
  std::map<std::string, std::string> bb_ports = {
    {"command", "command_with_rather_impossible_name"}, {"timeout", "1.0"}};
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::CommandHandler>("CommandHandler"));
  ASSERT_NO_THROW(CreateTree("CommandHandler", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCommandHandler, CommandReturnsFailure)
{
  std::map<std::string, std::string> bb_ports = {
    {"command", "echo 'Test command' && false"}, {"timeout", "1.0"}};
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::CommandHandler>("CommandHandler"));
  ASSERT_NO_THROW(CreateTree("CommandHandler", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCommandHandler, MultipleCommands)
{
  std::map<std::string, std::string> bb_ports = {
    {"command", "echo 'Test command' && echo 'Test command 2'"}, {"timeout", "1.0"}};
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::CommandHandler>("CommandHandler"));
  ASSERT_NO_THROW(CreateTree("CommandHandler", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCommandHandler, CommandTimeout)
{
  std::map<std::string, std::string> bb_ports = {{"command", "sleep infinity"}, {"timeout", "0.1"}};
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::CommandHandler>("CommandHandler"));
  ASSERT_NO_THROW(CreateTree("CommandHandler", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCommandHandler, TreeHalted)
{
  std::map<std::string, std::string> bb_ports = {{"command", "sleep infinity"}, {"timeout", "1.0"}};
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::CommandHandler>("CommandHandler"));
  ASSERT_NO_THROW(CreateTree("CommandHandler", bb_ports));

  auto & tree = GetTree();
  auto status = tree.tickOnce();

  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_NO_THROW(tree.haltTree());
  EXPECT_TRUE(tree.rootNode()->isHalted());
}
