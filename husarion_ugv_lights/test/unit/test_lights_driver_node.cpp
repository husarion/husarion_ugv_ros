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

#include <thread>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "husarion_ugv_msgs/srv/set_led_brightness.hpp"

#include "husarion_ugv_lights/apa102.hpp"
#include "husarion_ugv_lights/lights_driver_node.hpp"
#include "husarion_ugv_utils/test/ros_test_utils.hpp"

using ImageMsg = sensor_msgs::msg::Image;
using SetBoolSrv = std_srvs::srv::SetBool;
using SetLEDBrightnessSrv = husarion_ugv_msgs::srv::SetLEDBrightness;

class MockAPA102 : public husarion_ugv_lights::APA102Interface
{
public:
  MOCK_METHOD(void, SetGlobalBrightness, (const std::uint8_t brightness), (override));
  MOCK_METHOD(void, SetGlobalBrightness, (const float brightness), (override));
  MOCK_METHOD(void, SetPanel, (const std::vector<std::uint8_t> & frame), (const, override));

  using NiceMock = testing::NiceMock<MockAPA102>;
};

// LightsDriverNode constructor implemented for testing purposes
husarion_ugv_lights::LightsDriverNode::LightsDriverNode(
  APA102Interface::SharedPtr channel_1, APA102Interface::SharedPtr channel_2,
  const rclcpp::NodeOptions & options)
: Node("lights_driver", options),
  led_control_granted_(false),
  led_control_pending_(false),
  initialization_attempt_(0),
  channel_1_(channel_1),
  channel_2_(channel_2),
  diagnostic_updater_(this)
{
  channel_1_num_led_ = 46;
  channel_2_num_led_ = 46;
  frame_timeout_ = 0.1;
};

class DriverNodeWrapper : public husarion_ugv_lights::LightsDriverNode
{
public:
  DriverNodeWrapper(
    std::shared_ptr<MockAPA102> channel_1, std::shared_ptr<MockAPA102> channel_2,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions().use_intra_process_comms(true))
  : LightsDriverNode(channel_1, channel_2, options)
  {
  }

  void ClearLEDs() { return LightsDriverNode::ClearLEDs(); }

  void ToggleLEDControlCB(rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future)
  {
    return LightsDriverNode::ToggleLEDControlCB(future);
  }

  void FrameCB(
    const ImageMsg::UniquePtr & msg, const husarion_ugv_lights::APA102Interface::SharedPtr & panel,
    const rclcpp::Time & last_time, const std::string & panel_name)
  {
    return LightsDriverNode::FrameCB(msg, panel, last_time, panel_name);
  }

  int GetNumLed(const std::string & panel_name) const
  {
    return panel_name == "channel_1" ? channel_1_num_led_ : channel_2_num_led_;
  }
  double GetTimeout() const { return frame_timeout_; }
  bool GetLedControlGranted() const { return led_control_granted_; }
  bool GetLedControlPending() const { return led_control_pending_; }

  rclcpp::Time SetChannel1Ts(const rclcpp::Time & ts) { return channel_1_ts_ = ts; }
  rclcpp::Time SetChannel2Ts(const rclcpp::Time & ts) { return channel_2_ts_ = ts; }
};

class TestLightsDriverNode : public testing::Test
{
protected:
  TestLightsDriverNode();
  ~TestLightsDriverNode() {};

  ImageMsg::UniquePtr CreateValidImageMsg(const std::string & panel_name);
  std::shared_future<std::pair<SetBoolSrv::Request::SharedPtr, SetBoolSrv::Response::SharedPtr>>
  CreateSetBoolSrvFuture(bool request_data, bool response_success);

  std::shared_ptr<MockAPA102> channel_1_;
  std::shared_ptr<MockAPA102> channel_2_;
  std::unique_ptr<DriverNodeWrapper> lights_driver_node_;
};

TestLightsDriverNode::TestLightsDriverNode()
{
  channel_1_ = std::make_shared<MockAPA102::NiceMock>();
  channel_2_ = std::make_shared<MockAPA102::NiceMock>();

  lights_driver_node_ = std::make_unique<DriverNodeWrapper>(channel_1_, channel_2_);
}

ImageMsg::UniquePtr TestLightsDriverNode::CreateValidImageMsg(const std::string & panel_name)
{
  ImageMsg::UniquePtr msg(new ImageMsg);

  // Filling the message with valid data
  msg->header.stamp = lights_driver_node_->now();
  msg->header.frame_id = "image_frame";
  msg->height = 1;
  msg->width = lights_driver_node_->GetNumLed(panel_name);
  msg->encoding = sensor_msgs::image_encodings::RGBA8;
  msg->is_bigendian = false;
  msg->step = msg->width * 4;
  msg->data = std::vector<uint8_t>(msg->step * msg->height, 255);

  return msg;
}

std::shared_future<std::pair<SetBoolSrv::Request::SharedPtr, SetBoolSrv::Response::SharedPtr>>
TestLightsDriverNode::CreateSetBoolSrvFuture(bool request_data, bool response_success)
{
  auto request = std::make_shared<SetBoolSrv::Request>();
  request->data = request_data;

  auto response = std::make_shared<SetBoolSrv::Response>();
  response->success = response_success;

  std::promise<std::pair<SetBoolSrv::Request::SharedPtr, SetBoolSrv::Response::SharedPtr>> promise;
  promise.set_value(std::make_pair(request, response));

  return promise.get_future();
}

TEST_F(TestLightsDriverNode, TestInitialization)
{
  auto channel_1 = std::make_shared<MockAPA102>();
  auto channel_2 = std::make_shared<MockAPA102>();
  auto lights_driver_node = std::make_shared<DriverNodeWrapper>(channel_1, channel_2);

  EXPECT_TRUE(lights_driver_node != nullptr);
}

TEST_F(TestLightsDriverNode, ClearLEDs)
{
  auto num_led_1 = lights_driver_node_->GetNumLed("channel_1");
  auto num_led_2 = lights_driver_node_->GetNumLed("channel_2");
  std::vector<std::uint8_t> zero_frame_1(num_led_1 * 4, 0);
  std::vector<std::uint8_t> zero_frame_2(num_led_2 * 4, 0);

  EXPECT_CALL(*channel_1_, SetPanel(zero_frame_1)).Times(1);
  EXPECT_CALL(*channel_2_, SetPanel(zero_frame_2)).Times(1);

  lights_driver_node_->ClearLEDs();
}

// ### ToggleLEDControlCB tests ###

TEST_F(TestLightsDriverNode, ToggleLEDControlCBFailure)
{
  auto future = CreateSetBoolSrvFuture(true, false);

  lights_driver_node_->ToggleLEDControlCB(std::move(future));

  EXPECT_FALSE(lights_driver_node_->GetLedControlPending());
  EXPECT_FALSE(lights_driver_node_->GetLedControlGranted());
}

TEST_F(TestLightsDriverNode, ToggleLEDControlCBEnabled)
{
  auto future = CreateSetBoolSrvFuture(true, true);

  lights_driver_node_->ToggleLEDControlCB(std::move(future));

  EXPECT_FALSE(lights_driver_node_->GetLedControlPending());
  EXPECT_TRUE(lights_driver_node_->GetLedControlGranted());
}

TEST_F(TestLightsDriverNode, ToggleLEDControlCBDisabled)
{
  auto future = CreateSetBoolSrvFuture(false, true);

  lights_driver_node_->ToggleLEDControlCB(std::move(future));

  EXPECT_FALSE(lights_driver_node_->GetLedControlPending());
  EXPECT_FALSE(lights_driver_node_->GetLedControlGranted());
}

// ### FrameCB tests ###

TEST_F(TestLightsDriverNode, FrameCBSuccessNoControl)
{
  auto msg = CreateValidImageMsg("channel_1");

  EXPECT_CALL(*channel_1_, SetPanel(testing::_)).Times(0);

  lights_driver_node_->FrameCB(msg, channel_1_, msg->header.stamp, "channel_1");
}

TEST_F(TestLightsDriverNode, FrameCBSuccess)
{
  auto msg_1 = CreateValidImageMsg("channel_1");
  auto msg_2 = CreateValidImageMsg("channel_2");

  auto future = CreateSetBoolSrvFuture(true, true);
  lights_driver_node_->ToggleLEDControlCB(std::move(future));

  EXPECT_CALL(*channel_1_, SetPanel(testing::_)).Times(1);
  EXPECT_CALL(*channel_2_, SetPanel(testing::_)).Times(1);

  lights_driver_node_->FrameCB(msg_1, channel_1_, msg_1->header.stamp, "channel_1");
  lights_driver_node_->FrameCB(msg_2, channel_2_, msg_2->header.stamp, "channel_2");
}

TEST_F(TestLightsDriverNode, FrameCBTimeout)
{
  auto msg = CreateValidImageMsg("channel_1");
  auto timeout = lights_driver_node_->GetTimeout();

  // Set timestamp to exceed timeout
  msg->header.stamp.sec = msg->header.stamp.sec - timeout - 1;

  auto future = CreateSetBoolSrvFuture(true, true);
  lights_driver_node_->ToggleLEDControlCB(std::move(future));

  EXPECT_CALL(*channel_1_, SetPanel(msg->data)).Times(0);

  lights_driver_node_->FrameCB(msg, channel_1_, msg->header.stamp, "channel_1");
}

TEST_F(TestLightsDriverNode, FrameCBPast)
{
  auto msg = CreateValidImageMsg("channel_1");

  // Set last_time to be younger than msg timestamp
  auto future = CreateSetBoolSrvFuture(true, true);
  lights_driver_node_->ToggleLEDControlCB(std::move(future));

  EXPECT_CALL(*channel_1_, SetPanel(msg->data)).Times(0);

  lights_driver_node_->FrameCB(msg, channel_1_, lights_driver_node_->now(), "channel_1");
}

TEST_F(TestLightsDriverNode, FrameCBEncoding)
{
  auto msg = CreateValidImageMsg("channel_1");

  // Set incorrect encoding
  msg->encoding = sensor_msgs::image_encodings::RGB8;

  auto future = CreateSetBoolSrvFuture(true, true);
  lights_driver_node_->ToggleLEDControlCB(std::move(future));

  EXPECT_CALL(*channel_1_, SetPanel(msg->data)).Times(0);

  lights_driver_node_->FrameCB(msg, channel_1_, msg->header.stamp, "channel_1");
}

TEST_F(TestLightsDriverNode, FrameCBHeight)
{
  auto msg = CreateValidImageMsg("channel_1");

  // Set incorrect height
  msg->height = 2;

  auto future = CreateSetBoolSrvFuture(true, true);
  lights_driver_node_->ToggleLEDControlCB(std::move(future));

  EXPECT_CALL(*channel_1_, SetPanel(msg->data)).Times(0);

  lights_driver_node_->FrameCB(msg, channel_1_, msg->header.stamp, "channel_1");
}

TEST_F(TestLightsDriverNode, FrameCBWidth)
{
  auto msg = CreateValidImageMsg("channel_1");

  // Set incorrect width
  msg->width = lights_driver_node_->GetNumLed("channel_1") + 1;

  auto future = CreateSetBoolSrvFuture(true, true);
  lights_driver_node_->ToggleLEDControlCB(std::move(future));

  EXPECT_CALL(*channel_1_, SetPanel(msg->data)).Times(0);

  lights_driver_node_->FrameCB(msg, channel_1_, msg->header.stamp, "channel_1");
}

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
