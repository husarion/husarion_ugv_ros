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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include "husarion_ugv_manager/plugins/condition/check_string_msg.hpp"
#include "utils/plugin_test_utils.hpp"

using StringMsg = std_msgs::msg::String;
using bt_ports = std::map<std::string, std::string>;

struct TestCase
{
  BT::NodeStatus result;
  bt_ports input;
  StringMsg msg;
};

constexpr auto TOPIC = "string";
constexpr auto PLUGIN = "CheckStringMsg";

class TestCheckStringMsg : public husarion_ugv_manager::plugin_test_utils::PluginTestUtils
{
public:
  TestCheckStringMsg();
  StringMsg CreateMsg(const std::string & data);
  void PublishMsg(StringMsg msg) { publisher_->publish(msg); }

protected:
  rclcpp::Publisher<StringMsg>::SharedPtr publisher_;
};

TestCheckStringMsg::TestCheckStringMsg()
{
  RegisterNodeWithParams<husarion_ugv_manager::CheckStringMsg>(PLUGIN);
  publisher_ = bt_node_->create_publisher<StringMsg>(TOPIC, 10);
}

StringMsg TestCheckStringMsg::CreateMsg(const std::string & data)
{
  StringMsg msg;
  msg.data = data;
  return msg;
}

TEST_F(TestCheckStringMsg, NoMessageArrived)
{
  bt_ports input = {{"topic_name", TOPIC}, {"data", "name"}};
  ASSERT_NO_THROW({ CreateTree(PLUGIN, input); });

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCheckStringMsg, SuccessOnExactMatch)
{
  bt_ports input = {{"topic_name", TOPIC}, {"data", "name"}};
  CreateTree(PLUGIN, input);
  auto & tree = GetTree();
  auto status = tree.tickOnce();

  PublishMsg(CreateMsg("name"));
  status = tree.tickWhileRunning();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCheckStringMsg, SuccessOnMatchWithSpaces)
{
  bt_ports input = {{"topic_name", TOPIC}, {"data", "name with spaces"}};
  CreateTree(PLUGIN, input);
  auto & tree = GetTree();
  auto status = tree.tickOnce();

  PublishMsg(CreateMsg("name with spaces"));
  status = tree.tickWhileRunning();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCheckStringMsg, SuccessOnMatchWithDots)
{
  bt_ports input = {{"topic_name", TOPIC}, {"data", "name.with.dots"}};
  CreateTree(PLUGIN, input);
  auto & tree = GetTree();
  auto status = tree.tickOnce();

  PublishMsg(CreateMsg("name.with.dots"));
  status = tree.tickWhileRunning();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCheckStringMsg, SuccessOnMatchWithMixedLetters)
{
  bt_ports input = {{"topic_name", TOPIC}, {"data", "MiXeDlEtTeRs"}};
  CreateTree(PLUGIN, input);
  auto & tree = GetTree();
  auto status = tree.tickOnce();

  PublishMsg(CreateMsg("MiXeDlEtTeRs"));
  status = tree.tickWhileRunning();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCheckStringMsg, SuccessWhenValueChanges)
{
  bt_ports input = {{"topic_name", TOPIC}, {"data", "name"}};
  CreateTree(PLUGIN, input);
  auto & tree = GetTree();
  auto status = tree.tickOnce();

  PublishMsg(CreateMsg("name"));
  status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  PublishMsg(CreateMsg("name changed"));
  status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  PublishMsg(CreateMsg("name"));
  status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
