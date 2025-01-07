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

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_manager/plugins/action/signal_shutdown_node.hpp"
#include "utils/plugin_test_utils.hpp"

typedef husarion_ugv_manager::plugin_test_utils::PluginTestUtils TestSignalShutdown;

TEST_F(TestSignalShutdown, GoodLoadingSignalShutdownPlugin)
{
  std::map<std::string, std::string> service = {{"reason", "Test shutdown."}};

  RegisterNodeWithoutParams<husarion_ugv_manager::SignalShutdown>("SignalShutdown");

  ASSERT_NO_THROW({ CreateTree("SignalShutdown", service); });
}

TEST_F(TestSignalShutdown, WrongPluginNameLoadingSignalShutdownPlugin)
{
  std::map<std::string, std::string> service = {};

  EXPECT_THROW({ CreateTree("WrongSignalShutdown", service); }, BT::RuntimeError);
}

TEST_F(TestSignalShutdown, GoodCheckReasonBlackboardValue)
{
  std::map<std::string, std::string> service = {{"reason", "Test shutdown."}};

  RegisterNodeWithoutParams<husarion_ugv_manager::SignalShutdown>("SignalShutdown");

  CreateTree("SignalShutdown", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);

  auto blackboard = tree.rootBlackboard();
  auto signal_shutdown_value = blackboard->get<std::pair<bool, std::string>>("signal_shutdown");

  EXPECT_EQ(signal_shutdown_value.first, true);
  EXPECT_EQ(signal_shutdown_value.second, service["reason"]);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
