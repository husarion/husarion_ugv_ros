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
#include <fstream>
#include <map>
#include <string>

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_manager/plugins/action/shutdown_hosts_from_file_node.hpp"
#include "utils/plugin_test_utils.hpp"

typedef husarion_ugv_manager::plugin_test_utils::PluginTestUtils TestShutdownHostsFromFile;

TEST_F(TestShutdownHostsFromFile, GoodLoadingShutdownHostsFromFilePlugin)
{
  const std::map<std::string, std::string> service = {{"shutdown_hosts_file", "dummy_file"}};

  RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");

  ASSERT_NO_THROW({ CreateTree("ShutdownHostsFromFile", service); });
}

TEST_F(TestShutdownHostsFromFile, WrongPluginNameLoadingShutdownHostsFromFilePlugin)
{
  const std::map<std::string, std::string> service = {{"shutdown_hosts_file", "dummy_file"}};

  RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");

  EXPECT_THROW({ CreateTree("WrongShutdownHostsFromFile", service); }, BT::RuntimeError);
}

TEST_F(TestShutdownHostsFromFile, WrongCannotFindFileShutdownHostsFromFile)
{
  const std::string file_path =
    testing::TempDir() +
    "test_husarion_ugv_manager_wrong_cannot_find_file_shutdown_hosts_from_file";
  const std::map<std::string, std::string> service = {{"shutdown_hosts_file", file_path}};

  RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");

  CreateTree("ShutdownHostsFromFile", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestShutdownHostsFromFile, GoodShutdownHostsFromFile)
{
  const std::string config_file_path =
    testing::TempDir() + "test_husarion_ugv_manager_good_shutdown_hosts_from_file_config";
  const std::string test_file_path = testing::TempDir() +
                                     "test_husarion_ugv_manager_good_shutdown_hosts_from_file";
  std::filesystem::remove(test_file_path);
  std::filesystem::remove(config_file_path);

  EXPECT_FALSE(std::filesystem::exists(test_file_path));
  EXPECT_FALSE(std::filesystem::exists(config_file_path));

  YAML::Node shutdown_host_desc;
  shutdown_host_desc["hosts"][0]["ip"] = "localhost";
  shutdown_host_desc["hosts"][0]["username"] = "husarion";
  shutdown_host_desc["hosts"][0]["port"] = 22;

  shutdown_host_desc["hosts"][0]["command"] = "touch " + test_file_path;
  shutdown_host_desc["hosts"][0]["timeout"] = 5.0;
  shutdown_host_desc["hosts"][0]["ping_for_success"] = false;
  std::fstream config_file;
  YAML::Emitter emitter(config_file);

  config_file.open(config_file_path, std::ios::app);
  emitter << shutdown_host_desc;

  config_file.close();

  const std::map<std::string, std::string> service = {{"shutdown_hosts_file", config_file_path}};
  RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");

  auto & tree = GetTree();
  CreateTree("ShutdownHostsFromFile", service);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  std::filesystem::remove(test_file_path);
  std::filesystem::remove(config_file_path);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
