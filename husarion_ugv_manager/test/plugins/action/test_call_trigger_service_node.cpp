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

#include "husarion_ugv_manager/plugins/action/call_trigger_service_node.hpp"
#include "utils/plugin_test_utils.hpp"

class TestCallTriggerService : public husarion_ugv_manager::plugin_test_utils::PluginTestUtils
{
public:
  void ServiceCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /* request */,
    std_srvs::srv::Trigger::Response::SharedPtr response, const bool success);
};

void TestCallTriggerService::ServiceCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr /* request */,
  std_srvs::srv::Trigger::Response::SharedPtr response, const bool success)
{
  response->message = success ? "Successfully callback pass!" : "Failed callback pass!";
  response->success = success;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_trigger_plugin"), response->message);
}

TEST_F(TestCallTriggerService, GoodLoadingCallTriggerServicePlugin)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallTriggerService>("CallTriggerService");

  ASSERT_NO_THROW({ CreateTree("CallTriggerService", service); });
}

TEST_F(TestCallTriggerService, WrongPluginNameLoadingCallTriggerServicePlugin)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallTriggerService>("CallTriggerService");

  EXPECT_THROW({ CreateTree("WrongCallTriggerService", service); }, BT::RuntimeError);
}

TEST_F(TestCallTriggerService, WrongCallTriggerServiceServiceServerNotInitialized)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallTriggerService>("CallTriggerService");

  CreateTree("CallTriggerService", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCallTriggerService, GoodTriggerCallServiceSuccess)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallTriggerService>("CallTriggerService");

  CreateTree("CallTriggerService", service);
  auto & tree = GetTree();

  using std_srvs::srv::Trigger;

  CreateService<Trigger>(
    "trigger",
    [&](const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response) {
      ServiceCallback(request, response, true);
    });

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCallTriggerService, WrongTriggerCallServiceFailure)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallTriggerService>("CallTriggerService");

  CreateTree("CallTriggerService", service);
  auto & tree = GetTree();

  using std_srvs::srv::Trigger;

  CreateService<Trigger>(
    "trigger",
    [&](const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response) {
      ServiceCallback(request, response, false);
    });

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
