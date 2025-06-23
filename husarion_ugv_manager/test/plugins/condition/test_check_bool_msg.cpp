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
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include "husarion_ugv_manager/plugins/condition/check_bool_msg.hpp"
#include "utils/plugin_test_utils.hpp"

using BoolMsg = std_msgs::msg::Bool;
using bt_ports = std::map<std::string, std::string>;

struct TestCase
{
  BT::NodeStatus result;
  bt_ports input;
  BoolMsg msg;
};

constexpr auto TOPIC = "bool";
constexpr auto PLUGIN = "CheckBoolMsg";

class TestCheckBoolMsg : public husarion_ugv_manager::plugin_test_utils::PluginTestUtils
{
public:
  TestCheckBoolMsg();
  BoolMsg CreateMsg(bool data);
  void PublishMsg(BoolMsg msg) { publisher_->publish(msg); }

protected:
  rclcpp::Publisher<BoolMsg>::SharedPtr publisher_;
};

TestCheckBoolMsg::TestCheckBoolMsg()
{
  RegisterNodeWithParams<husarion_ugv_manager::CheckBoolMsg>(PLUGIN);
  publisher_ = bt_node_->create_publisher<BoolMsg>(TOPIC, 10);
}

BoolMsg TestCheckBoolMsg::CreateMsg(bool data)
{
  BoolMsg msg;
  msg.data = data;
  return msg;
}

TEST_F(TestCheckBoolMsg, NoMessageArrived)
{
  bt_ports input = {{"topic_name", TOPIC}, {"data", "true"}};
  ASSERT_NO_THROW({ CreateTree(PLUGIN, input); });

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCheckBoolMsg, OnTickBehavior)
{
  std::vector<TestCase> test_cases = {
    {BT::NodeStatus::SUCCESS, {{"topic_name", TOPIC}, {"data", "true"}}, CreateMsg(true)},
    {BT::NodeStatus::SUCCESS, {{"topic_name", TOPIC}, {"data", "false"}}, CreateMsg(false)},
    {BT::NodeStatus::FAILURE, {{"topic_name", TOPIC}, {"data", "true"}}, CreateMsg(false)},
    {BT::NodeStatus::FAILURE, {{"topic_name", TOPIC}, {"data", "false"}}, CreateMsg(true)}};

  for (auto & test_case : test_cases) {
    CreateTree(PLUGIN, test_case.input);
    auto & tree = GetTree();
    auto status = tree.tickOnce();

    PublishMsg(test_case.msg);
    status = tree.tickWhileRunning();

    EXPECT_EQ(status, test_case.result);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
