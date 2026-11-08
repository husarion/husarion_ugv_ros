// Copyright 2026 Husarion sp. z o.o.
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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_msgs/msg/battery_protection.hpp"
#include "husarion_ugv_msgs/msg/led_animation.hpp"
#include "husarion_ugv_msgs/srv/set_led_animation.hpp"

#include <husarion_ugv_manager/lights_manager_node.hpp>

using BatteryProtectionMsg = husarion_ugv_msgs::msg::BatteryProtection;
using LEDAnimationMsg = husarion_ugv_msgs::msg::LEDAnimation;
using SetLEDAnimationSrv = husarion_ugv_msgs::srv::SetLEDAnimation;

class TestLightsManagerUVP : public testing::Test
{
protected:
  TestLightsManagerUVP()
  {
    bt_project_path_ = testing::TempDir() + "test_lights_uvp_bt.btproj";
    std::ofstream(bt_project_path_) << R"(
      <root BTCPP_format="4" project_name="Test">
        <BehaviorTree ID="Lights">
          <AlwaysSuccess/>
        </BehaviorTree>
      </root>
    )";

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("bt_project_path", bt_project_path_),
      rclcpp::Parameter("plugin_libs", std::vector<std::string>{}),
      rclcpp::Parameter("ros_plugin_libs", std::vector<std::string>{}),
      rclcpp::Parameter("uvp_enable", true),
    });
    node_ = std::make_shared<husarion_ugv_manager::LightsManagerNode>("test_lights_uvp", options);
    set_animation_server_ = node_->create_service<SetLEDAnimationSrv>(
      "lights/set_animation", [this](
                                const SetLEDAnimationSrv::Request::SharedPtr request,
                                SetLEDAnimationSrv::Response::SharedPtr response) {
        requests_.push_back(*request);
        response->success = true;
      });
    node_->Initialize();
    protection_pub_ = node_->create_publisher<BatteryProtectionMsg>(
      "battery/protection", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  }

  ~TestLightsManagerUVP() { std::filesystem::remove(bt_project_path_); }

  // Publishes the state every second, as the battery driver does, while spinning.
  void PublishFor(const BatteryProtectionMsg & msg, const std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    auto next_publish = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
      if (std::chrono::steady_clock::now() >= next_publish) {
        protection_pub_->publish(msg);
        next_publish += std::chrono::milliseconds(1000);
      }
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  static BatteryProtectionMsg ChargeAsap(const bool reactions_enabled)
  {
    BatteryProtectionMsg msg;
    msg.state = BatteryProtectionMsg::STATE_CHARGE_ASAP;
    msg.reactions_enabled = reactions_enabled;
    return msg;
  }

  std::string bt_project_path_;
  std::shared_ptr<husarion_ugv_manager::LightsManagerNode> node_;
  rclcpp::Service<SetLEDAnimationSrv>::SharedPtr set_animation_server_;
  rclcpp::Publisher<BatteryProtectionMsg>::SharedPtr protection_pub_;
  std::vector<SetLEDAnimationSrv::Request> requests_;
};

TEST_F(TestLightsManagerUVP, ChargeAsapPlaysWithReactionsEnabled)
{
  EXPECT_EQ(1u, node_->count_subscribers("/battery/protection"));
  PublishFor(ChargeAsap(true), std::chrono::milliseconds(1500));
  ASSERT_EQ(1u, requests_.size());
  EXPECT_EQ(LEDAnimationMsg::CHARGE_ASAP, requests_[0].animation.id);
  EXPECT_FALSE(requests_[0].repeating);
}

TEST_F(TestLightsManagerUVP, ChargeAsapStaysOffWithReactionsDisabled)
{
  PublishFor(ChargeAsap(false), std::chrono::milliseconds(1500));
  EXPECT_TRUE(requests_.empty());
}

TEST_F(TestLightsManagerUVP, NothingWhileOkOrUvp)
{
  auto msg = ChargeAsap(true);
  msg.state = BatteryProtectionMsg::STATE_OK;
  PublishFor(msg, std::chrono::milliseconds(1000));
  msg.state = BatteryProtectionMsg::STATE_UVP;
  PublishFor(msg, std::chrono::milliseconds(1000));
  EXPECT_TRUE(requests_.empty());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
