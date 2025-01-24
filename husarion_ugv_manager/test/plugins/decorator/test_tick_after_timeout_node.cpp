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

#include "husarion_ugv_manager/plugins/decorator/tick_after_timeout_node.hpp"
#include "utils/plugin_test_utils.hpp"

class TestTickAfterTimeout : public husarion_ugv_manager::plugin_test_utils::PluginTestUtils
{
public:
  virtual std::string BuildBehaviorTree(
    const std::string & plugin_name, const std::map<std::string, std::string> & service) override;
};

std::string TestTickAfterTimeout::BuildBehaviorTree(
  const std::string & /* plugin_name */, const std::map<std::string, std::string> & /* service */
)
{
  std::stringstream bt;

  bt << tree_header_ << std::endl;
  bt << "\t\t\t<TickAfterTimeout timeout=\"0.1\" >" << std::endl;

  bt << "\t\t\t\t<AlwaysSuccess name=\"success_action\"/>" << std::endl;

  bt << "\t\t\t</TickAfterTimeout>" << std::endl;

  bt << tree_footer_;

  return bt.str();
}

TEST_F(TestTickAfterTimeout, GoodTickAfterTimeout)
{
  RegisterNodeWithoutParams<husarion_ugv_manager::TickAfterTimeout>("TickAfterTimeout");
  CreateTree("", {});
  auto & tree = GetTree();

  // node has just started the timeout should not be reached
  auto status = tree.tickOnce();
  EXPECT_EQ(BT::NodeStatus::SKIPPED, status);

  // sleep for timeout duration
  // then the node should return child status - AlwaysSuccess
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  status = tree.tickOnce();
  EXPECT_EQ(BT::NodeStatus::SUCCESS, status);

  // tick again to check if timeout was reset
  status = tree.tickOnce();
  EXPECT_EQ(BT::NodeStatus::SKIPPED, status);

  // check again if timeout is correctly evaluated
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  status = tree.tickOnce();
  EXPECT_EQ(BT::NodeStatus::SUCCESS, status);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
