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
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "husarion_ugv_msgs/msg/battery_protection.hpp"
#include "husarion_ugv_msgs/msg/io_state.hpp"

#include "husarion_ugv_manager/safety_manager_node.hpp"

using BatteryProtectionMsg = husarion_ugv_msgs::msg::BatteryProtection;
using IOStateMsg = husarion_ugv_msgs::msg::IOState;
using TriggerSrv = std_srvs::srv::Trigger;

namespace
{

std::vector<std::string> & CallOrder()
{
  static std::vector<std::string> order;
  return order;
}

// Stand-ins for the plugins the UVP shutdown tree uses, recording what they were given.
class FakeShutdownHosts : public BT::SyncActionNode
{
public:
  FakeShutdownHosts(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("shutdown_hosts_file"), BT::InputPort<float>("timeout_override")};
  }
  BT::NodeStatus tick() override
  {
    const auto timeout = getInput<float>("timeout_override");
    CallOrder().push_back(
      "hosts:" + getInput<std::string>("shutdown_hosts_file").value() + ":" +
      std::to_string(timeout.value()));
    // A failed host must not stop the sequence.
    return BT::NodeStatus::FAILURE;
  }
};

class FakeSetBool : public BT::SyncActionNode
{
public:
  FakeSetBool(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("service_name"), BT::InputPort<bool>("data")};
  }
  BT::NodeStatus tick() override
  {
    CallOrder().push_back(
      "set_bool:" + getInput<std::string>("service_name").value() + ":" +
      (getInput<bool>("data").value() ? "true" : "false"));
    return BT::NodeStatus::FAILURE;
  }
};

class FakeExecuteCommand : public BT::SyncActionNode
{
public:
  FakeExecuteCommand(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("command"), BT::InputPort<float>("timeout")};
  }
  BT::NodeStatus tick() override
  {
    CallOrder().push_back("command:" + getInput<std::string>("command").value());
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

class SafetyManagerUVPWrapper : public husarion_ugv_manager::SafetyManagerNode
{
public:
  SafetyManagerUVPWrapper(const std::string & node_name, const rclcpp::NodeOptions & options)
  : SafetyManagerNode(node_name, options)
  {
  }

  using SafetyManagerNode::kUVPShutdownTreeXML;
  using SafetyManagerNode::UseUVPShutdown;

  bool UVPShutdownTreeReady() const { return uvp_shutdown_tree_manager_ != nullptr; }

  struct ShutdownCall
  {
    std::string reason;
    bool uvp_requested;
    bool uvp_sequence;
  };
  std::vector<ShutdownCall> shutdown_calls;

protected:
  // Records instead of powering the machine off.
  void ShutdownRobot(const std::string & reason, const bool uvp) override
  {
    shutdown_calls.push_back({reason, uvp, UseUVPShutdown(uvp)});
  }
};

TEST(TestUVPShutdownTree, HostsThenAuxThenPowerOff)
{
  CallOrder().clear();
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<FakeShutdownHosts>("ShutdownHostsFromFile");
  factory.registerNodeType<FakeSetBool>("CallSetBoolService");
  factory.registerNodeType<FakeExecuteCommand>("ExecuteCommand");
  factory.registerBehaviorTreeFromText(SafetyManagerUVPWrapper::kUVPShutdownTreeXML);

  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::string>("SHUTDOWN_HOSTS_FILE", "/hosts.yaml");
  blackboard->set<std::string>("SHUTDOWN_LOCALHOST_COMMAND", "poweroff");
  blackboard->set<float>("UVP_HOSTS_TIMEOUT", 3.0f);
  auto tree = factory.createTree("UVPShutdown", blackboard);
  EXPECT_EQ(BT::NodeStatus::SUCCESS, tree.tickWhileRunning());

  ASSERT_EQ(3u, CallOrder().size());
  EXPECT_EQ("hosts:/hosts.yaml:" + std::to_string(3.0f), CallOrder()[0]);
  EXPECT_EQ("set_bool:hardware/aux_power_enable:false", CallOrder()[1]);
  EXPECT_EQ("command:poweroff", CallOrder()[2]);
}

class TestSafetyManagerUVP : public testing::Test
{
protected:
  void Create(const bool uvp_enable, const bool with_uvp_plugins = true)
  {
    bt_project_path_ = testing::TempDir() + "test_uvp_bt.btproj";
    std::ofstream(bt_project_path_) << R"(
      <root BTCPP_format="4" project_name="Test">
        <BehaviorTree ID="Safety">
          <AlwaysSuccess/>
        </BehaviorTree>
        <BehaviorTree ID="Shutdown">
          <AlwaysSuccess/>
        </BehaviorTree>
      </root>
    )";

    std::vector<std::string> plugin_libs = {
      "tick_after_timeout_bt_node", "signal_shutdown_bt_node"};
    if (with_uvp_plugins) {
      plugin_libs.push_back("execute_command_bt_node");
      plugin_libs.push_back("shutdown_hosts_from_file_bt_node");
    }
    const std::vector<std::string> ros_plugin_libs = {
      "call_set_bool_service_bt_node", "call_trigger_service_bt_node"};

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("bt_project_path", bt_project_path_),
      rclcpp::Parameter("plugin_libs", plugin_libs),
      rclcpp::Parameter("ros_plugin_libs", ros_plugin_libs),
      rclcpp::Parameter("groot_enabled", false),
      rclcpp::Parameter("uvp_enable", uvp_enable),
    });
    node_ = std::make_shared<SafetyManagerUVPWrapper>("test_safety_manager_uvp", options);
    e_stop_trigger_server_ = node_->create_service<TriggerSrv>(
      "hardware/e_stop_trigger",
      [](const TriggerSrv::Request::SharedPtr, TriggerSrv::Response::SharedPtr) {});
    node_->Initialize();

    protection_pub_ = node_->create_publisher<BatteryProtectionMsg>(
      "battery/protection", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    io_state_pub_ = node_->create_publisher<IOStateMsg>(
      "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  }

  void TearDown() override { std::filesystem::remove(bt_project_path_); }

  static BatteryProtectionMsg Uvp(const std::uint8_t mode, const float stop_timeout_min)
  {
    BatteryProtectionMsg msg;
    msg.state = BatteryProtectionMsg::STATE_UVP;
    msg.mode = mode;
    msg.reactions_enabled = true;
    msg.stop_timeout_min = stop_timeout_min;
    msg.shutdown_hosts_timeout_s = 3.0f;
    msg.trip_id = 1;
    msg.trip_stamp.sec = 1000;
    msg.seconds_since_trip = 0.0f;
    return msg;
  }

  bool SpinUntil(const std::function<bool()> & pred, const std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  std::string bt_project_path_;
  std::shared_ptr<SafetyManagerUVPWrapper> node_;
  rclcpp::Service<TriggerSrv>::SharedPtr e_stop_trigger_server_;
  rclcpp::Publisher<BatteryProtectionMsg>::SharedPtr protection_pub_;
  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
};

TEST_F(TestSafetyManagerUVP, DisabledChangesNothing)
{
  Create(false);
  protection_pub_->publish(Uvp(BatteryProtectionMsg::MODE_SHUTDOWN, 5.0f));
  EXPECT_FALSE(
    SpinUntil([this]() { return !node_->shutdown_calls.empty(); }, std::chrono::seconds(1)));
  EXPECT_EQ(0u, node_->count_subscribers("/battery/protection"));
  EXPECT_FALSE(node_->UVPShutdownTreeReady());
  EXPECT_FALSE(node_->UseUVPShutdown(true));

  // The power button shuts down as before.
  IOStateMsg io_state;
  io_state.power_button = true;
  io_state_pub_->publish(io_state);
  ASSERT_TRUE(
    SpinUntil([this]() { return !node_->shutdown_calls.empty(); }, std::chrono::seconds(2)));
  EXPECT_FALSE(node_->shutdown_calls[0].uvp_sequence);
}

TEST_F(TestSafetyManagerUVP, ShutdownModeShutsDownAtTheTrip)
{
  Create(true);
  EXPECT_TRUE(node_->UVPShutdownTreeReady());
  EXPECT_EQ(1u, node_->count_subscribers("/battery/protection"));

  protection_pub_->publish(Uvp(BatteryProtectionMsg::MODE_SHUTDOWN, 5.0f));
  ASSERT_TRUE(
    SpinUntil([this]() { return !node_->shutdown_calls.empty(); }, std::chrono::seconds(2)));
  EXPECT_TRUE(node_->shutdown_calls[0].uvp_requested);
  EXPECT_TRUE(node_->shutdown_calls[0].uvp_sequence);
}

TEST_F(TestSafetyManagerUVP, StopModeShutsDownAfterT)
{
  Create(true);
  // T of 0.6 s.
  const auto msg = Uvp(BatteryProtectionMsg::MODE_STOP, 0.01f);
  const auto start = std::chrono::steady_clock::now();
  protection_pub_->publish(msg);
  ASSERT_TRUE(
    SpinUntil([this]() { return !node_->shutdown_calls.empty(); }, std::chrono::seconds(3)));
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(550));
  EXPECT_TRUE(node_->shutdown_calls[0].uvp_sequence);
  EXPECT_EQ(1u, node_->shutdown_calls.size());
}

TEST_F(TestSafetyManagerUVP, ChargingHoldsTheShutdown)
{
  Create(true);
  auto msg = Uvp(BatteryProtectionMsg::MODE_STOP, 0.01f);
  msg.charging = true;
  protection_pub_->publish(msg);
  EXPECT_FALSE(
    SpinUntil([this]() { return !node_->shutdown_calls.empty(); }, std::chrono::seconds(2)));
}

TEST_F(TestSafetyManagerUVP, PowerButtonWhileLatchedTakesTheUVPSequence)
{
  Create(true);
  protection_pub_->publish(Uvp(BatteryProtectionMsg::MODE_STOP, 5.0f));
  ASSERT_TRUE(
    SpinUntil([this]() { return node_->UseUVPShutdown(false); }, std::chrono::seconds(2)));

  IOStateMsg io_state;
  io_state.power_button = true;
  io_state_pub_->publish(io_state);
  ASSERT_TRUE(
    SpinUntil([this]() { return !node_->shutdown_calls.empty(); }, std::chrono::seconds(2)));
  EXPECT_EQ("Power button pressed", node_->shutdown_calls[0].reason);
  EXPECT_FALSE(node_->shutdown_calls[0].uvp_requested);
  EXPECT_TRUE(node_->shutdown_calls[0].uvp_sequence);
}

TEST_F(TestSafetyManagerUVP, ReactionsOffKeepTheNormalSequence)
{
  Create(true);
  auto msg = Uvp(BatteryProtectionMsg::MODE_SHUTDOWN, 998.0f);
  msg.reactions_enabled = false;
  protection_pub_->publish(msg);
  EXPECT_FALSE(
    SpinUntil([this]() { return !node_->shutdown_calls.empty(); }, std::chrono::seconds(1)));
  EXPECT_FALSE(node_->UseUVPShutdown(false));

  IOStateMsg io_state;
  io_state.power_button = true;
  io_state_pub_->publish(io_state);
  ASSERT_TRUE(
    SpinUntil([this]() { return !node_->shutdown_calls.empty(); }, std::chrono::seconds(2)));
  EXPECT_FALSE(node_->shutdown_calls[0].uvp_sequence);
}

TEST_F(TestSafetyManagerUVP, MissingPluginsFallBackToTheNormalSequence)
{
  Create(true, false);
  EXPECT_FALSE(node_->UVPShutdownTreeReady());
  EXPECT_FALSE(node_->UseUVPShutdown(true));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
