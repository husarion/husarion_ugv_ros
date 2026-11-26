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

#include "utils/test_battery_driver_node.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "husarion_ugv_msgs/msg/battery_protection.hpp"

#include "husarion_ugv_battery/protection/protection_publisher.hpp"
#include "husarion_ugv_battery/protection/uvp_detector.hpp"
#include "husarion_ugv_utils/test/ros_test_utils.hpp"

using BatteryProtectionMsg = husarion_ugv_msgs::msg::BatteryProtection;
using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

// Raw ADC values for the test fixture's scale of 1.0 on the voltage channels.
constexpr int kRawVoltageOk = 1400;   // 35.06 V
constexpr int kRawVoltageUvp = 1160;  // 29.05 V

std::filesystem::path LatchPath()
{
  return std::filesystem::path(testing::TempDir()) / "uvp_latch_test";
}

std::vector<rclcpp::Parameter> UvpParams(std::vector<rclcpp::Parameter> extra = {})
{
  extra.push_back(rclcpp::Parameter("uvp.enable", true));
  // A path that never exists, so the threshold is the default on any machine running the test.
  extra.push_back(rclcpp::Parameter("uvp.hat_eeprom_path", "/nonexistent/hat/custom_0"));
  return extra;
}

}  // namespace

// Subscriptions and helpers shared by the protection tests.
class TestBatteryProtection
{
protected:
  void Subscribe(const rclcpp::Node::SharedPtr & node)
  {
    protection_sub_ = node->create_subscription<BatteryProtectionMsg>(
      "battery/protection", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const BatteryProtectionMsg::SharedPtr msg) { protection_ = msg; });
    diagnostics_sub_ = node->create_subscription<DiagnosticArrayMsg>(
      "/diagnostics", 10, [this](const DiagnosticArrayMsg::SharedPtr msg) {
        for (const auto & status : msg->status) {
          // Skip the status the updater sends when a task is added.
          if (
            status.name.find("Battery protection") != std::string::npos &&
            status.message != "Node starting up") {
            protection_status_ = std::make_shared<DiagnosticStatusMsg>(status);
          }
        }
      });
  }

  // Spins until the predicate holds for the latest protection message.
  bool WaitForProtection(
    const rclcpp::Node::SharedPtr & node,
    const std::function<bool(const BatteryProtectionMsg &)> & pred,
    const std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      if (protection_ && pred(*protection_)) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  bool WaitForDiagnostic(
    const rclcpp::Node::SharedPtr & node, const std::chrono::milliseconds timeout)
  {
    protection_status_.reset();
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      if (protection_status_) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  static std::string ValueOf(const DiagnosticStatusMsg & status, const std::string & key)
  {
    for (const auto & kv : status.values) {
      if (kv.key == key) {
        return kv.value;
      }
    }
    return "<missing>";
  }

  BatteryProtectionMsg::SharedPtr protection_;
  std::shared_ptr<DiagnosticStatusMsg> protection_status_;
  rclcpp::Subscription<BatteryProtectionMsg>::SharedPtr protection_sub_;
  rclcpp::Subscription<DiagnosticArrayMsg>::SharedPtr diagnostics_sub_;
};

class TestBatteryNodeUvpOff : public TestBatteryNode, public TestBatteryProtection
{
public:
  TestBatteryNodeUvpOff() : TestBatteryNode(true, false) {}
};

TEST_F(TestBatteryNodeUvpOff, NoProtectionWhenDisabled)
{
  Subscribe(battery_driver_node_);
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(5000)));

  // A low voltage changes nothing either.
  WriteNumberToFile<int>(kRawVoltageUvp, device1_path_ / "in_voltage0_raw");
  std::vector<std::string> task_names;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  auto diagnostics_sub = battery_driver_node_->create_subscription<DiagnosticArrayMsg>(
    "/diagnostics", 10, [&task_names](const DiagnosticArrayMsg::SharedPtr msg) {
      for (const auto & status : msg->status) {
        task_names.push_back(status.name);
      }
    });
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(battery_driver_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(0u, battery_driver_node_->count_publishers("/test_battery/battery/protection"));
  EXPECT_FALSE(protection_);
  ASSERT_FALSE(task_names.empty());
  for (const auto & name : task_names) {
    EXPECT_EQ(std::string::npos, name.find("Battery protection")) << name;
  }

  // The parameters exist and are inert.
  EXPECT_FALSE(battery_driver_node_->get_parameter("uvp.enable").as_bool());
}

class TestBatteryNodeUvpSingle : public TestBatteryNode, public TestBatteryProtection
{
public:
  TestBatteryNodeUvpSingle() : TestBatteryNode(true, false, UvpParams()) {}
};

TEST_F(TestBatteryNodeUvpSingle, TripsOnTwoLowReads)
{
  Subscribe(battery_driver_node_);
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_OK; },
    std::chrono::seconds(5)));

  EXPECT_EQ(1u, battery_driver_node_->count_publishers("/test_battery/battery/protection"));
  EXPECT_TRUE(protection_->reactions_enabled);
  EXPECT_EQ(BatteryProtectionMsg::MODE_STOP, protection_->mode);
  EXPECT_FLOAT_EQ(31.0f, protection_->uvp_threshold);
  EXPECT_FLOAT_EQ(31.5f, protection_->charge_asap_threshold);
  EXPECT_EQ("default", protection_->threshold_source);
  EXPECT_FLOAT_EQ(5.0f, protection_->stop_timeout_min);
  EXPECT_FLOAT_EQ(60.0f, protection_->drive_window_s);
  EXPECT_FLOAT_EQ(10.0f, protection_->shutdown_hosts_timeout_s);
  ASSERT_EQ(1u, protection_->battery_states.size());
  EXPECT_EQ(0u, protection_->trip_id);

  WriteNumberToFile<int>(kRawVoltageUvp, device1_path_ / "in_voltage0_raw");
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_UVP; },
    std::chrono::seconds(2)));
  EXPECT_EQ(1u, protection_->trip_id);
  ASSERT_EQ(1u, protection_->trip_voltage.size());
  EXPECT_NEAR(29.05f, protection_->trip_voltage[0], 0.01f);
  EXPECT_FALSE(std::isnan(protection_->trip_current[0]));
  EXPECT_GE(protection_->seconds_since_trip, 0.0);
  EXPECT_GE(protection_->seconds_without_charging, 0.0);

  // Back above the threshold without charging: still latched.
  WriteNumberToFile<int>(kRawVoltageOk, device1_path_ / "in_voltage0_raw");
  EXPECT_FALSE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state != BatteryProtectionMsg::STATE_UVP; },
    std::chrono::seconds(2)));

  ASSERT_TRUE(WaitForDiagnostic(battery_driver_node_, std::chrono::seconds(3)));
  EXPECT_EQ(DiagnosticStatusMsg::ERROR, protection_status_->level);
  EXPECT_EQ("UVP", ValueOf(*protection_status_, "State"));
  EXPECT_EQ("default", ValueOf(*protection_status_, "Threshold source"));
}

TEST_F(TestBatteryNodeUvpSingle, PublishesOnceASecond)
{
  Subscribe(battery_driver_node_);
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_OK; },
    std::chrono::seconds(5)));
  const rclcpp::Time first(protection_->header.stamp);
  protection_.reset();
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_, [](const BatteryProtectionMsg &) { return true; },
    std::chrono::milliseconds(1500)));
  EXPECT_GT(rclcpp::Time(protection_->header.stamp), first);
}

class TestBatteryNodeUvpParam : public TestBatteryNode, public TestBatteryProtection
{
public:
  TestBatteryNodeUvpParam()
  : TestBatteryNode(
      true, false,
      UvpParams(
        {rclcpp::Parameter("uvp.threshold_v", "33.0"),
         rclcpp::Parameter("uvp.stop_timeout_min", 998)}))
  {
  }
};

TEST_F(TestBatteryNodeUvpParam, OverrideAndReactionsOffWarn)
{
  Subscribe(battery_driver_node_);
  // 35.06 V is above the overridden charge ASAP threshold of 33.5 V.
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_OK; },
    std::chrono::seconds(5)));
  EXPECT_EQ("param", protection_->threshold_source);
  EXPECT_FLOAT_EQ(33.0f, protection_->uvp_threshold);
  EXPECT_FALSE(protection_->reactions_enabled);
  EXPECT_EQ("UVP reactions disabled by configuration", protection_->reactions_disabled_reason);

  // Detection still runs with the reactions off.
  WriteNumberToFile<int>(kRawVoltageUvp, device1_path_ / "in_voltage0_raw");
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_UVP; },
    std::chrono::seconds(2)));

  WriteNumberToFile<int>(kRawVoltageOk, device1_path_ / "in_voltage0_raw");
  ASSERT_TRUE(WaitForDiagnostic(battery_driver_node_, std::chrono::seconds(3)));
  EXPECT_EQ("param", ValueOf(*protection_status_, "Threshold source"));
  EXPECT_EQ("UVP reactions disabled by configuration", ValueOf(*protection_status_, "Reactions"));
}

class TestBatteryNodeUvpInvalidParam : public TestBatteryNode, public TestBatteryProtection
{
public:
  TestBatteryNodeUvpInvalidParam()
  : TestBatteryNode(true, false, UvpParams({rclcpp::Parameter("uvp.drive_window_s", "abc")}))
  {
  }
};

TEST_F(TestBatteryNodeUvpInvalidParam, InvalidParamIsAnErrorNotACrash)
{
  Subscribe(battery_driver_node_);
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_OK; },
    std::chrono::seconds(5)));
  EXPECT_FLOAT_EQ(60.0f, protection_->drive_window_s);
  EXPECT_TRUE(protection_->reactions_enabled);

  ASSERT_TRUE(WaitForDiagnostic(battery_driver_node_, std::chrono::seconds(3)));
  EXPECT_EQ(DiagnosticStatusMsg::ERROR, protection_status_->level);
  EXPECT_NE(std::string::npos, protection_status_->message.find("uvp.drive_window_s"));
}

class TestBatteryNodeUvpDual : public TestBatteryNode, public TestBatteryProtection
{
public:
  TestBatteryNodeUvpDual() : TestBatteryNode(true, true, UvpParams()) {}
};

TEST_F(TestBatteryNodeUvpDual, OneBatteryTripsTheRobot)
{
  Subscribe(battery_driver_node_);
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_OK; },
    std::chrono::seconds(10)));
  ASSERT_EQ(2u, protection_->battery_states.size());

  // Battery 2 voltage.
  WriteNumberToFile<int>(kRawVoltageUvp, device1_path_ / "in_voltage3_raw");
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_UVP; },
    std::chrono::seconds(2)));
  EXPECT_EQ(BatteryProtectionMsg::STATE_OK, protection_->battery_states[0]);
  EXPECT_EQ(BatteryProtectionMsg::STATE_UVP, protection_->battery_states[1]);
  EXPECT_TRUE(std::isnan(protection_->trip_voltage[0]));
  EXPECT_NEAR(29.05f, protection_->trip_voltage[1], 0.01f);
}

TEST_F(TestBatteryNodeUvpDual, ThrowingBatteryOneStillFeedsBothVoltages)
{
  Subscribe(battery_driver_node_);
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_OK; },
    std::chrono::seconds(10)));

  // Battery 1 current read now throws, after its voltage was read.
  std::filesystem::remove(device1_path_ / "in_voltage2_raw");
  // Battery 1 voltage low: its read is fed although the rest of its update fails.
  WriteNumberToFile<int>(kRawVoltageUvp, device1_path_ / "in_voltage0_raw");
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) {
      return msg.battery_states.size() == 2 &&
             msg.battery_states[0] == BatteryProtectionMsg::STATE_UVP;
    },
    std::chrono::seconds(2)));

  // Battery 2 is still updated after battery 1 throws.
  WriteNumberToFile<int>(kRawVoltageUvp, device1_path_ / "in_voltage3_raw");
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) {
      return msg.battery_states[1] == BatteryProtectionMsg::STATE_UVP;
    },
    std::chrono::seconds(2)));
  EXPECT_EQ(1u, protection_->trip_id);
}

class TestBatteryNodeUvpRoboteq : public TestBatteryNode, public TestBatteryProtection
{
public:
  TestBatteryNodeUvpRoboteq() : TestBatteryNode(false, false, UvpParams()) {}
};

TEST_F(TestBatteryNodeUvpRoboteq, DetectsButNoReactions)
{
  Subscribe(battery_driver_node_);
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_, [](const BatteryProtectionMsg &) { return true; },
    std::chrono::seconds(10)));
  EXPECT_FALSE(protection_->reactions_enabled);
  EXPECT_EQ("reactions off: charging cannot be detected", protection_->reactions_disabled_reason);

  const auto publish_voltage = [this](const float voltage) {
    husarion_ugv_msgs::msg::DriverStateNamed motor_controller;
    motor_controller.state.voltage = voltage;
    motor_controller.state.current = 1.0f;
    RobotDriverStateMsg driver_state;
    driver_state.header.stamp = battery_driver_node_->get_clock()->now();
    driver_state.driver_states.push_back(motor_controller);
    driver_state.driver_states.push_back(motor_controller);
    driver_state_pub_->publish(driver_state);
  };

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    publish_voltage(29.0f);
    rclcpp::spin_some(battery_driver_node_);
    if (protection_ && protection_->state == BatteryProtectionMsg::STATE_UVP) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(protection_);
  EXPECT_EQ(BatteryProtectionMsg::STATE_UVP, protection_->state);
  EXPECT_FALSE(protection_->reactions_enabled);
  EXPECT_NEAR(2.0f, protection_->trip_current[0], 0.01f);
}

class TestBatteryNodeUvpLatch : public TestBatteryNode, public TestBatteryProtection
{
public:
  TestBatteryNodeUvpLatch()
  : TestBatteryNode(
      true, false,
      (WriteLatch(), UvpParams({rclcpp::Parameter("uvp.latch_path", LatchPath().string())})))
  {
  }

  ~TestBatteryNodeUvpLatch() { std::filesystem::remove(LatchPath()); }

  static void WriteLatch()
  {
    timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    husarion_ugv_battery::UvpLatch latch;
    latch.boot_id = husarion_ugv_battery::ProtectionPublisher::ReadBootId();
    latch.trip_id = 7;
    latch.trip_boottime_ns = static_cast<std::int64_t>(ts.tv_sec - 90) * 1000000000LL;
    // Charged after the trip, and the charge ended 30 s ago.
    latch.countdown_boottime_ns = static_cast<std::int64_t>(ts.tv_sec - 30) * 1000000000LL;
    latch.trip_stamp_ns = 1700000000LL * 1000000000LL;
    latch.battery_uvp = {true};
    latch.trip_voltage = {29.4f};
    latch.trip_current = {11.0f};
    std::ofstream(LatchPath()) << husarion_ugv_battery::SerializeUvpLatch(latch);
  }
};

TEST_F(TestBatteryNodeUvpLatch, RestartKeepsTheLatchAndTheElapsedTime)
{
  Subscribe(battery_driver_node_);
  // The voltage is fine, the latch is what holds UVP.
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_, [](const BatteryProtectionMsg &) { return true; },
    std::chrono::seconds(5)));
  EXPECT_EQ(BatteryProtectionMsg::STATE_UVP, protection_->state);
  EXPECT_EQ(7u, protection_->trip_id);
  EXPECT_FLOAT_EQ(29.4f, protection_->trip_voltage[0]);
  EXPECT_GE(protection_->seconds_since_trip, 89.0);
  EXPECT_GE(protection_->seconds_without_charging, 29.0);
  EXPECT_LT(protection_->seconds_without_charging, 60.0);
  EXPECT_EQ(1700000000, protection_->trip_stamp.sec);

  EXPECT_FALSE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state != BatteryProtectionMsg::STATE_UVP; },
    std::chrono::seconds(2)));
}

std::filesystem::path LatchDirLater()
{
  return std::filesystem::path(testing::TempDir()) / "uvp_latch_dir_later";
}

class TestBatteryNodeUvpLatchRetry : public TestBatteryNode, public TestBatteryProtection
{
public:
  TestBatteryNodeUvpLatchRetry()
  : TestBatteryNode(
      true, false,
      (std::filesystem::remove_all(LatchDirLater()),
       UvpParams({rclcpp::Parameter("uvp.latch_path", (LatchDirLater() / "latch").string())})))
  {
  }

  ~TestBatteryNodeUvpLatchRetry() { std::filesystem::remove_all(LatchDirLater()); }
};

TEST_F(TestBatteryNodeUvpLatchRetry, FailedLatchWriteIsRetried)
{
  Subscribe(battery_driver_node_);
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_OK; },
    std::chrono::seconds(5)));

  // The trip cannot be written while the directory is missing.
  WriteNumberToFile<int>(kRawVoltageUvp, device1_path_ / "in_voltage0_raw");
  ASSERT_TRUE(WaitForProtection(
    battery_driver_node_,
    [](const BatteryProtectionMsg & msg) { return msg.state == BatteryProtectionMsg::STATE_UVP; },
    std::chrono::seconds(2)));
  EXPECT_FALSE(WaitForProtection(
    battery_driver_node_, [](const BatteryProtectionMsg &) { return false; },
    std::chrono::milliseconds(500)));
  EXPECT_FALSE(std::filesystem::exists(LatchDirLater() / "latch"));

  // No further change of the latch: the same trip is written once the directory exists.
  std::filesystem::create_directories(LatchDirLater());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline &&
         !std::filesystem::exists(LatchDirLater() / "latch")) {
    rclcpp::spin_some(battery_driver_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(std::filesystem::exists(LatchDirLater() / "latch"));
  std::ifstream file(LatchDirLater() / "latch");
  const std::string content(
    (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(1u, husarion_ugv_battery::ParseUvpLatch(content).trip_id);
  EXPECT_TRUE(husarion_ugv_battery::ParseUvpLatch(content).battery_uvp.at(0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
