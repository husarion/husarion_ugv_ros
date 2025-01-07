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

#include "husarion_ugv_manager/plugins/action/call_set_led_animation_service_node.hpp"
#include "utils/plugin_test_utils.hpp"

class TestCallSetLedAnimationService
: public husarion_ugv_manager::plugin_test_utils::PluginTestUtils
{
public:
  void ServiceCallback(
    const husarion_ugv_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
    husarion_ugv_msgs::srv::SetLEDAnimation::Response::SharedPtr response, const bool success,
    const std::size_t id, const bool repeating);
};

void TestCallSetLedAnimationService::ServiceCallback(
  const husarion_ugv_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
  husarion_ugv_msgs::srv::SetLEDAnimation::Response::SharedPtr response, const bool success,
  const std::size_t id, const bool repeating)
{
  response->message = success ? "Successfully callback pass!" : "Failed callback pass!";
  response->success = success;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_set_led_animation_plugin"),
    response->message << " success: " << response->success << " id: " << request->animation.id
                      << " param: " << request->animation.param
                      << " repeating: " << request->repeating);

  EXPECT_EQ(request->animation.id, id);
  EXPECT_EQ(request->repeating, repeating);
}

TEST_F(TestCallSetLedAnimationService, GoodLoadingCallSetLedAnimationServicePlugin)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  ASSERT_NO_THROW({ CreateTree("CallSetLedAnimationService", service); });
}

TEST_F(TestCallSetLedAnimationService, WrongPluginNameLoadingCallSetLedAnimationServicePlugin)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  EXPECT_THROW({ CreateTree("WrongCallSetLedAnimationService", service); }, BT::RuntimeError);
}

TEST_F(TestCallSetLedAnimationService, WrongCallSetLedAnimationServiceServiceServerNotInitialized)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCallSetLedAnimationService, GoodSetLedAnimationCallServiceSuccessWithTrueRepeatingValue)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using husarion_ugv_msgs::srv::SetLEDAnimation;

  CreateService<SetLEDAnimation>(
    "set_led_animation", [&](
                           const SetLEDAnimation::Request::SharedPtr request,
                           SetLEDAnimation::Response::SharedPtr response) {
      ServiceCallback(request, response, true, 0, true);
    });
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCallSetLedAnimationService, GoodSetLedAnimationCallServiceSuccessWithFalseRepeatingValue)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "false"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using husarion_ugv_msgs::srv::SetLEDAnimation;

  CreateService<SetLEDAnimation>(
    "set_led_animation", [&](
                           const SetLEDAnimation::Request::SharedPtr request,
                           SetLEDAnimation::Response::SharedPtr response) {
      ServiceCallback(request, response, true, 0, false);
    });
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCallSetLedAnimationService, GoodSetLedAnimationCallServiceSuccessWith_5IdValue)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "5"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using husarion_ugv_msgs::srv::SetLEDAnimation;

  CreateService<SetLEDAnimation>(
    "set_led_animation", [&](
                           const SetLEDAnimation::Request::SharedPtr request,
                           SetLEDAnimation::Response::SharedPtr response) {
      ServiceCallback(request, response, true, 5, true);
    });
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCallSetLedAnimationService, WrongSetLedAnimationCallServiceFailure)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<husarion_ugv_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using husarion_ugv_msgs::srv::SetLEDAnimation;

  CreateService<SetLEDAnimation>(
    "set_led_animation", [&](
                           const SetLEDAnimation::Request::SharedPtr request,
                           SetLEDAnimation::Response::SharedPtr response) {
      ServiceCallback(request, response, false, 0, true);
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
