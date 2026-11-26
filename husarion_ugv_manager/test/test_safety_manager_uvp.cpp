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

#include <atomic>
#include <chrono>
#include <cmath>
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

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "husarion_ugv_msgs/msg/battery_protection.hpp"
#include "husarion_ugv_msgs/msg/io_state.hpp"

#include "husarion_ugv_manager/safety_manager_node.hpp"

using BatteryProtectionMsg = husarion_ugv_msgs::msg::BatteryProtection;
using IOStateMsg = husarion_ugv_msgs::msg::IOState;
using SetBoolSrv = std_srvs::srv::SetBool;
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

  // Stand-ins for the plugins of both shutdown trees, so the real ShutdownRobot runs without
  // powering anything off. Called before Initialize, in place of the plugin libraries.
  // Without the UVP plugins only the command stand-in is there, so the UVP tree cannot be built.
  void RegisterFakeShutdownPlugins(const bool with_uvp_plugins)
  {
    factory_.registerNodeType<FakeExecuteCommand>("ExecuteCommand");
    if (with_uvp_plugins) {
      factory_.registerNodeType<FakeShutdownHosts>("ShutdownHostsFromFile");
      factory_.registerNodeType<FakeSetBool>("CallSetBoolService");
    }
  }

  // Short SHUTDOWN mode confirm and stale times, so the tests need not wait 10 s.
  void SetUvpReactionTimes(
    const std::chrono::milliseconds confirm,
    const double stale_timeout_s = husarion_ugv_manager::UvpReaction::kDefaultStaleTimeoutS)
  {
    uvp_reaction_ = std::make_unique<husarion_ugv_manager::UvpReaction>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(confirm).count(), stale_timeout_s);
  }

  diagnostic_msgs::msg::DiagnosticStatus DiagnoseUVP()
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    SafetyManagerNode::DiagnoseUVP(status);
    return status;
  }

  int stops = 0;

protected:
  // The shutdown tree has run. The test process keeps its ROS context.
  void StopNode() override { ++stops; }
};

TEST(TestUVPShutdownTree, HostsThenAux)
{
  CallOrder().clear();
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<FakeShutdownHosts>("ShutdownHostsFromFile");
  factory.registerNodeType<FakeSetBool>("CallSetBoolService");
  factory.registerBehaviorTreeFromText(SafetyManagerUVPWrapper::kUVPShutdownTreeXML);

  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::string>("SHUTDOWN_HOSTS_FILE", "/hosts.yaml");
  blackboard->set<float>("UVP_HOSTS_TIMEOUT", 3.0f);
  auto tree = factory.createTree("UVPShutdown", blackboard);
  EXPECT_EQ(BT::NodeStatus::SUCCESS, tree.tickWhileRunning());

  ASSERT_EQ(2u, CallOrder().size());
  EXPECT_EQ("hosts:/hosts.yaml:" + std::to_string(3.0f), CallOrder()[0]);
  EXPECT_EQ("set_bool:hardware/aux_power_enable:false", CallOrder()[1]);
}

class TestSafetyManagerUVP : public testing::Test
{
protected:
  void Create(
    const bool uvp_enable, const bool with_uvp_plugins = true,
    const double service_response_timeout = 1.0)
  {
    CallOrder().clear();
    bt_project_path_ = testing::TempDir() + "test_uvp_bt.btproj";
    // The normal shutdown leaves a mark through the fake ExecuteCommand.
    const std::string shutdown_tree = R"(<ExecuteCommand command="normal" timeout="1.0"/>)";
    std::ofstream(bt_project_path_) << R"(
      <root BTCPP_format="4" project_name="Test">
        <BehaviorTree ID="Safety">
          <AlwaysSuccess/>
        </BehaviorTree>
        <BehaviorTree ID="Shutdown">
          )" << shutdown_tree << R"(
        </BehaviorTree>
      </root>
    )";

    const std::vector<std::string> plugin_libs = {
      "tick_after_timeout_bt_node", "signal_shutdown_bt_node"};
    std::vector<std::string> ros_plugin_libs = {"call_trigger_service_bt_node"};
    if (!with_uvp_plugins) {
      // The UVP tree still misses the hosts and the command plugins.
      ros_plugin_libs.push_back("call_set_bool_service_bt_node");
    }

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("bt_project_path", bt_project_path_),
      rclcpp::Parameter("plugin_libs", plugin_libs),
      rclcpp::Parameter("ros_plugin_libs", ros_plugin_libs),
      rclcpp::Parameter("uvp_enable", uvp_enable),
      rclcpp::Parameter("ros_communication_timeout.response", service_response_timeout),
    });
    node_ = std::make_shared<SafetyManagerUVPWrapper>("test_safety_manager_uvp", options);
    node_->RegisterFakeShutdownPlugins(with_uvp_plugins);
    e_stop_trigger_server_ = node_->create_service<TriggerSrv>(
      "hardware/e_stop_trigger",
      [](const TriggerSrv::Request::SharedPtr, TriggerSrv::Response::SharedPtr) {});
    node_->Initialize();

    protection_pub_ = node_->create_publisher<BatteryProtectionMsg>(
      "battery/protection", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    io_state_pub_ = node_->create_publisher<IOStateMsg>(
      "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  }

  // The first step of the shutdown tree that ran: the UVP sequence starts with the hosts.
  static bool RanUVPSequence()
  {
    return !CallOrder().empty() && CallOrder().front().rfind("hosts:", 0) == 0;
  }

  static bool RanNormalSequence()
  {
    return CallOrder() == std::vector<std::string>{"command:normal"};
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
    msg.seconds_since_trip = 0.0;
    msg.seconds_without_charging = 0.0;
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
  EXPECT_FALSE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(1)));
  EXPECT_EQ(0u, node_->count_subscribers("/battery/protection"));
  EXPECT_FALSE(node_->UVPShutdownTreeReady());
  EXPECT_FALSE(node_->UseUVPShutdown(true));

  // The power button shuts down as before.
  IOStateMsg io_state;
  io_state.power_button = true;
  io_state_pub_->publish(io_state);
  ASSERT_TRUE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(2)));
  EXPECT_TRUE(RanNormalSequence());
}

TEST_F(TestSafetyManagerUVP, ShutdownModeShutsDownWithoutCharging)
{
  Create(true);
  node_->SetUvpReactionTimes(std::chrono::milliseconds(300));
  EXPECT_TRUE(node_->UVPShutdownTreeReady());
  EXPECT_EQ(1u, node_->count_subscribers("/battery/protection"));

  const auto start = std::chrono::steady_clock::now();
  protection_pub_->publish(Uvp(BatteryProtectionMsg::MODE_SHUTDOWN, 5.0f));
  ASSERT_TRUE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(2)));
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(250));
  EXPECT_TRUE(RanUVPSequence());
}

TEST_F(TestSafetyManagerUVP, ChargerConnectedHoldsTheShutdownInShutdownMode)
{
  Create(true);
  node_->SetUvpReactionTimes(std::chrono::milliseconds(300));
  // Not debounced yet by the battery driver.
  auto msg = Uvp(BatteryProtectionMsg::MODE_SHUTDOWN, 5.0f);
  msg.charger_connected = true;
  protection_pub_->publish(msg);
  EXPECT_FALSE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(1)));
}

TEST_F(TestSafetyManagerUVP, StopModeTurnsTheAuxPowerOffAtTheTrip)
{
  Create(true);
  std::atomic_int calls{0};
  bool requested_data = true;
  const auto aux_server = node_->create_service<SetBoolSrv>(
    "hardware/aux_power_enable",
    [&](const SetBoolSrv::Request::SharedPtr request, SetBoolSrv::Response::SharedPtr response) {
      requested_data = request->data;
      response->success = true;
      ++calls;
    });

  auto msg = Uvp(BatteryProtectionMsg::MODE_STOP, 5.0f);
  msg.aux_off_at_trip = true;
  protection_pub_->publish(msg);
  ASSERT_TRUE(SpinUntil([&calls]() { return calls > 0; }, std::chrono::seconds(2)));
  EXPECT_FALSE(requested_data);

  // Once per trip.
  protection_pub_->publish(msg);
  EXPECT_FALSE(SpinUntil([&calls]() { return calls > 1; }, std::chrono::milliseconds(500)));
  EXPECT_EQ(0, node_->stops);
}

TEST_F(TestSafetyManagerUVP, DiagnosticsReportTheLatchAndTheData)
{
  Create(true);
  auto status = node_->DiagnoseUVP();
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::OK, status.level);

  protection_pub_->publish(Uvp(BatteryProtectionMsg::MODE_STOP, 5.0f));
  ASSERT_TRUE(
    SpinUntil([this]() { return node_->UseUVPShutdown(false); }, std::chrono::seconds(2)));
  status = node_->DiagnoseUVP();
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::WARN, status.level);
  bool has_age = false;
  bool has_countdown = false;
  for (const auto & kv : status.values) {
    has_age = has_age || kv.key == "UVP data age (s)";
    has_countdown = has_countdown || kv.key == "Seconds to UVP shutdown";
  }
  EXPECT_TRUE(has_age);
  EXPECT_TRUE(has_countdown);
}

TEST_F(TestSafetyManagerUVP, StopModeShutsDownAfterT)
{
  Create(true);
  // T of 0.6 s.
  const auto msg = Uvp(BatteryProtectionMsg::MODE_STOP, 0.01f);
  const auto start = std::chrono::steady_clock::now();
  protection_pub_->publish(msg);
  ASSERT_TRUE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(3)));
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(550));
  EXPECT_TRUE(RanUVPSequence());
  EXPECT_EQ(1, node_->stops);
}

TEST_F(TestSafetyManagerUVP, ChargingHoldsTheShutdown)
{
  Create(true);
  auto msg = Uvp(BatteryProtectionMsg::MODE_STOP, 0.01f);
  msg.charging = true;
  protection_pub_->publish(msg);
  EXPECT_FALSE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(2)));
}

TEST_F(TestSafetyManagerUVP, ChargingHoldsTheShutdownInShutdownMode)
{
  Create(true);
  auto msg = Uvp(BatteryProtectionMsg::MODE_SHUTDOWN, 5.0f);
  msg.charging = true;
  protection_pub_->publish(msg);
  EXPECT_FALSE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(2)));
  EXPECT_TRUE(node_->UseUVPShutdown(false));
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
  ASSERT_TRUE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(2)));
  EXPECT_TRUE(RanUVPSequence());
}

TEST_F(TestSafetyManagerUVP, ReactionsOffKeepTheNormalSequence)
{
  Create(true);
  auto msg = Uvp(BatteryProtectionMsg::MODE_SHUTDOWN, 998.0f);
  msg.reactions_enabled = false;
  protection_pub_->publish(msg);
  EXPECT_FALSE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(1)));
  EXPECT_FALSE(node_->UseUVPShutdown(false));

  IOStateMsg io_state;
  io_state.power_button = true;
  io_state_pub_->publish(io_state);
  ASSERT_TRUE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(2)));
  EXPECT_TRUE(RanNormalSequence());
}

TEST_F(TestSafetyManagerUVP, MissingPluginsFallBackToTheNormalSequence)
{
  Create(true, false);
  EXPECT_FALSE(node_->UVPShutdownTreeReady());
  EXPECT_FALSE(node_->UseUVPShutdown(true));
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::ERROR, node_->DiagnoseUVP().level);

  // A UVP shutdown still happens, through the normal tree.
  node_->SetUvpReactionTimes(std::chrono::milliseconds(300));
  protection_pub_->publish(Uvp(BatteryProtectionMsg::MODE_SHUTDOWN, 5.0f));
  ASSERT_TRUE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(3)));
  EXPECT_TRUE(RanNormalSequence());
  EXPECT_EQ(1, node_->stops);
}

TEST_F(TestSafetyManagerUVP, AuxOffAtTheTripWaitsForTheService)
{
  Create(true);
  // The trip comes before the hardware system offers the service, as at a boot on a flat battery.
  auto msg = Uvp(BatteryProtectionMsg::MODE_STOP, 5.0f);
  msg.aux_off_at_trip = true;
  protection_pub_->publish(msg);
  ASSERT_TRUE(
    SpinUntil([this]() { return node_->UseUVPShutdown(false); }, std::chrono::seconds(2)));
  SpinUntil([]() { return false; }, std::chrono::milliseconds(500));

  std::atomic_int calls{0};
  bool requested_data = true;
  const auto aux_server = node_->create_service<SetBoolSrv>(
    "hardware/aux_power_enable",
    [&](const SetBoolSrv::Request::SharedPtr request, SetBoolSrv::Response::SharedPtr response) {
      requested_data = request->data;
      response->success = true;
      ++calls;
    });
  ASSERT_TRUE(SpinUntil([&calls]() { return calls > 0; }, std::chrono::seconds(3)));
  EXPECT_FALSE(requested_data);
  // Once, after the success.
  EXPECT_FALSE(SpinUntil([&calls]() { return calls > 1; }, std::chrono::milliseconds(500)));
}

TEST_F(TestSafetyManagerUVP, UVPShutdownRunsHostsAndAuxThenTheUsersShutdownTree)
{
  Create(true);
  // T of 0.6 s. The tree starts with 10 s for the hosts; the message says 3 s.
  protection_pub_->publish(Uvp(BatteryProtectionMsg::MODE_STOP, 0.01f));
  ASSERT_TRUE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(3)));

  const auto hosts_file = node_->get_parameter("shutdown_hosts_path").as_string();
  ASSERT_EQ(3u, CallOrder().size());
  EXPECT_EQ("hosts:" + hosts_file + ":" + std::to_string(3.0f), CallOrder()[0]);
  EXPECT_EQ("set_bool:hardware/aux_power_enable:false", CallOrder()[1]);
  // The project's own Shutdown tree still runs, and powers the computer off.
  EXPECT_EQ("command:normal", CallOrder()[2]);

  // Once: the UVP timer is cancelled.
  EXPECT_FALSE(SpinUntil([this]() { return node_->stops > 1; }, std::chrono::milliseconds(500)));
}

TEST_F(TestSafetyManagerUVP, StaleChargingDataNoLongerHoldsTheShutdown)
{
  Create(true);
  node_->SetUvpReactionTimes(std::chrono::seconds(10), 0.3);
  // T of 0.6 s, sent once while charging, then the battery driver goes silent.
  auto msg = Uvp(BatteryProtectionMsg::MODE_STOP, 0.01f);
  msg.charging = true;
  msg.seconds_without_charging = std::nan("");
  const auto start = std::chrono::steady_clock::now();
  protection_pub_->publish(msg);
  ASSERT_TRUE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(3)));
  // T counts from the last message that said charging.
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(550));
  EXPECT_TRUE(RanUVPSequence());
}

TEST_F(TestSafetyManagerUVP, StaleChargerDataNoLongerHoldsTheShutdownInShutdownMode)
{
  Create(true);
  node_->SetUvpReactionTimes(std::chrono::milliseconds(300), 0.3);
  auto msg = Uvp(BatteryProtectionMsg::MODE_SHUTDOWN, 5.0f);
  msg.charger_connected = true;
  protection_pub_->publish(msg);
  ASSERT_TRUE(SpinUntil([this]() { return node_->stops > 0; }, std::chrono::seconds(3)));
  EXPECT_TRUE(RanUVPSequence());
}

TEST_F(TestSafetyManagerUVP, UnansweredAuxOffIsDroppedAndSentAgain)
{
  Create(true, true, 0.3);
  // A hardware system that takes the request and never answers it, as one that went away.
  std::atomic_int silent_calls{0};
  auto silent_server = node_->create_service<SetBoolSrv>(
    "hardware/aux_power_enable",
    [&](std::shared_ptr<rmw_request_id_t>, SetBoolSrv::Request::SharedPtr) { ++silent_calls; });

  auto msg = Uvp(BatteryProtectionMsg::MODE_STOP, 5.0f);
  msg.aux_off_at_trip = true;
  protection_pub_->publish(msg);
  ASSERT_TRUE(SpinUntil([&]() { return silent_calls > 1; }, std::chrono::seconds(3)));

  // Back: the next request is answered, and that ends the retries.
  silent_server.reset();
  std::atomic_int calls{0};
  const auto aux_server = node_->create_service<SetBoolSrv>(
    "hardware/aux_power_enable",
    [&](const SetBoolSrv::Request::SharedPtr, SetBoolSrv::Response::SharedPtr response) {
      response->success = true;
      ++calls;
    });
  ASSERT_TRUE(SpinUntil([&calls]() { return calls > 0; }, std::chrono::seconds(3)));
  EXPECT_FALSE(SpinUntil([&calls]() { return calls > 1; }, std::chrono::milliseconds(800)));
}

TEST_F(TestSafetyManagerUVP, RefusedAuxOffBacksOff)
{
  Create(true);
  std::atomic_int calls{0};
  auto refusing_server = node_->create_service<SetBoolSrv>(
    "hardware/aux_power_enable",
    [&](const SetBoolSrv::Request::SharedPtr, SetBoolSrv::Response::SharedPtr response) {
      response->success = false;
      response->message = "refused";
      ++calls;
    });

  auto msg = Uvp(BatteryProtectionMsg::MODE_STOP, 5.0f);
  msg.aux_off_at_trip = true;
  protection_pub_->publish(msg);
  ASSERT_TRUE(SpinUntil([&calls]() { return calls > 0; }, std::chrono::seconds(2)));
  const auto first = std::chrono::steady_clock::now();

  // Tried again after 1 s, then 2 s later: not at the 100 ms timer rate.
  ASSERT_TRUE(SpinUntil([&calls]() { return calls > 1; }, std::chrono::seconds(3)));
  EXPECT_GE(std::chrono::steady_clock::now() - first, std::chrono::milliseconds(900));
  SpinUntil([&calls]() { return calls > 2; }, std::chrono::milliseconds(1500));
  EXPECT_EQ(2, calls.load());

  // Accepted on a later try, which ends the retries.
  refusing_server.reset();
  std::atomic_int accepted{0};
  const auto aux_server = node_->create_service<SetBoolSrv>(
    "hardware/aux_power_enable",
    [&](const SetBoolSrv::Request::SharedPtr, SetBoolSrv::Response::SharedPtr response) {
      response->success = true;
      ++accepted;
    });
  ASSERT_TRUE(SpinUntil([&accepted]() { return accepted > 0; }, std::chrono::seconds(4)));
  EXPECT_FALSE(SpinUntil([&accepted]() { return accepted > 1; }, std::chrono::milliseconds(800)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
