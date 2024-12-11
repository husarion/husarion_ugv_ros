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
#include <filesystem>
#include <map>
#include <string>

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_manager/plugins/action/shutdown_single_host_node.hpp"
#include "utils/plugin_test_utils.hpp"

typedef husarion_ugv_manager::plugin_test_utils::PluginTestUtils TestShutdownSingleHost;

TEST_F(TestShutdownSingleHost, GoodLoadingShutdownSingleHostPlugin)
{
  std::map<std::string, std::string> service = {
    {"command", "pwd"}, {"ip", "localhost"}, {"ping_for_success", "false"},
    {"port", "22"},     {"timeout", "5.0"},  {"username", "husarion"},
  };

  RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownSingleHost>("ShutdownSingleHost");

  ASSERT_NO_THROW({ CreateTree("ShutdownSingleHost", service); });
}

TEST_F(TestShutdownSingleHost, WrongPluginNameLoadingShutdownSingleHostPlugin)
{
  std::map<std::string, std::string> service = {
    {"command", "pwd"}, {"ip", "localhost"}, {"ping_for_success", "false"},
    {"port", "22"},     {"timeout", "5.0"},  {"username", "husarion"},
  };

  RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownSingleHost>("ShutdownSingleHost");
  EXPECT_THROW({ CreateTree("WrongShutdownSingleHost", service); }, BT::RuntimeError);
}

TEST_F(TestShutdownSingleHost, GoodTouchCommand)
{
  std::string test_file_path = testing::TempDir() + "test_husarion_ugv_manager_good_touch_command";
  std::filesystem::remove(test_file_path);
  EXPECT_FALSE(std::filesystem::exists(test_file_path));
  std::map<std::string, std::string> service = {
    {"command", "touch " + test_file_path},
    {"ip", "localhost"},
    {"ping_for_success", "false"},
    {"port", "22"},
    {"timeout", "5.0"},
    {"username", "husarion"},
  };
  RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownSingleHost>("ShutdownSingleHost");
  CreateTree("ShutdownSingleHost", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(std::filesystem::exists(test_file_path));

  std::filesystem::remove(test_file_path);
}

TEST_F(TestShutdownSingleHost, Timeout)
{
  std::map<std::string, std::string> service = {
    {"command", "sleep 0.5"}, {"ip", "localhost"}, {"ping_for_success", "false"},
    {"port", "22"},           {"timeout", "0.1"},  {"username", "husarion"},
  };
  RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownSingleHost>("ShutdownSingleHost");
  CreateTree("ShutdownSingleHost", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestShutdownSingleHost, WrongUser)
{
  std::map<std::string, std::string> service = {
    {"command", "echo Hello World!"},
    {"ip", "localhost"},
    {"ping_for_success", "false"},
    {"port", "22"},
    {"timeout", "0.2"},
    {"username", "wrong_user"},
  };
  RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownSingleHost>("ShutdownSingleHost");

  CreateTree("ShutdownSingleHost", service);
  auto & tree = GetTree();
  CreateTree("ShutdownSingleHost", service);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
