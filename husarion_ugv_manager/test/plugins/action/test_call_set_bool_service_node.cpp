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
#include <string>

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_manager/plugins/action/call_set_bool_service_node.hpp"
#include "utils/plugin_test_utils.hpp"

class TestCallSetBoolService : public husarion_ugv_manager::plugin_test_utils::PluginTestUtils
{
public:
  void ServiceCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response, const bool success,
    const bool expected_value);
};

void TestCallSetBoolService::ServiceCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response, bool success, bool expected_value)
{
  response->message = success ? "Successfully callback pass!" : "Failed callback pass!";
  response->success = success;
  EXPECT_EQ(request->data, expected_value);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_set_bool_plugin"), response->message << " data: " << request->data);
}

TEST_F(TestCallSetBoolService, GoodLoadingCallSetBoolServicePlugin)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "true"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetBoolService>("CallSetBoolService");

  ASSERT_NO_THROW({ CreateTree("CallSetBoolService", service); });
}

TEST_F(TestCallSetBoolService, WrongPluginNameLoadingCallSetBoolServicePlugin)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "true"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetBoolService>("CallSetBoolService");

  EXPECT_THROW({ CreateTree("WrongCallSetBoolService", service); }, BT::RuntimeError);
}

TEST_F(TestCallSetBoolService, WrongCallSetBoolServiceServiceServerNotInitialized)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "true"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetBoolService>("CallSetBoolService");

  CreateTree("CallSetBoolService", service);

  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCallSetBoolService, GoodSetBoolCallServiceSuccessWithTrueValue)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "true"}};

  using std_srvs::srv::SetBool;
  CreateService<SetBool>(
    "set_bool",
    [&](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
      ServiceCallback(request, response, true, true);
    });

  RegisterNodeWithParams<husarion_ugv_manager::CallSetBoolService>("CallSetBoolService");

  CreateTree("CallSetBoolService", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCallSetBoolService, GoodSetBoolCallServiceSuccessWithFalseValue)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "false"}};

  using std_srvs::srv::SetBool;
  CreateService<SetBool>(
    "set_bool",
    [&](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
      ServiceCallback(request, response, true, false);
    });

  RegisterNodeWithParams<husarion_ugv_manager::CallSetBoolService>("CallSetBoolService");

  CreateTree("CallSetBoolService", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(1000));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCallSetBoolService, WrongSetBoolCallServiceFailure)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "false"}};

  using std_srvs::srv::SetBool;
  CreateService<SetBool>(
    "set_bool",
    [&](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
      ServiceCallback(request, response, false, false);
    });

  RegisterNodeWithParams<husarion_ugv_manager::CallSetBoolService>("CallSetBoolService");

  CreateTree("CallSetBoolService", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(1000));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
