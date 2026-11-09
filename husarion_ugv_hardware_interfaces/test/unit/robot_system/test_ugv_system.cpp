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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "husarion_ugv_msgs/msg/battery_protection.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/ugv_system.hpp"

#include "utils/system_test_utils.hpp"

class MockUGVSystem : public husarion_ugv_hardware_interfaces::UGVSystem
{
public:
  MockUGVSystem(const std::vector<std::string> & joint_order)
  : husarion_ugv_hardware_interfaces::UGVSystem(joint_order)
  {
    mock_robot_driver_ =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockRobotDriver::NiceMock>();
    mock_gpio_controller_ =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockGPIOController::NiceMock>();
    mock_e_stop_ = std::make_shared<husarion_ugv_hardware_interfaces_test::MockEStop::NiceMock>();

    ON_CALL(*this, DefineRobotDriver()).WillByDefault(::testing::Invoke([&]() {
      robot_driver_ = mock_robot_driver_;
    }));
    ON_CALL(*this, ConfigureGPIOController()).WillByDefault(::testing::Invoke([&]() {
      gpio_controller_ = mock_gpio_controller_;
    }));
    ON_CALL(*this, ConfigureEStop()).WillByDefault(::testing::Invoke([&]() {
      e_stop_ = mock_e_stop_;
    }));
  }

  MOCK_METHOD(void, ReadCANopenSettingsDriverCANIDs, (), (override));
  MOCK_METHOD(void, ConfigureGPIOController, (), (override));
  MOCK_METHOD(void, ConfigureEStop, (), (override));

  MOCK_METHOD(void, DefineRobotDriver, (), (override));

  MOCK_METHOD(void, UpdateHwStates, (), (override));
  MOCK_METHOD(bool, UpdateMotorsStateDataTimedOut, (), (override));
  MOCK_METHOD(void, UpdateDriverStateMsg, (), (override));
  MOCK_METHOD(void, UpdateFlagErrors, (), (override));
  MOCK_METHOD(void, UpdateDriverStateDataTimedOut, (), (override));

  MOCK_METHOD(std::vector<float>, GetSpeedCommands, (), (const, override));

  MOCK_METHOD(void, DiagnoseErrors, (diagnostic_updater::DiagnosticStatusWrapper &), (override));
  MOCK_METHOD(void, DiagnoseStatus, (diagnostic_updater::DiagnosticStatusWrapper &), (override));
  MOCK_METHOD(std::vector<DriverDiagnosticData>, GetDriversDiagnosticData, (), (override));

  void SetHwCommandVelocity(const std::vector<double> & velocity)
  {
    hw_commands_velocities_ = velocity;
  }
  void SetHwStatePosition(const std::vector<double> & position) { hw_states_positions_ = position; }
  void SetHwStateVelocity(const std::vector<double> & velocity)
  {
    hw_states_velocities_ = velocity;
  }
  void SetHwStateEffort(const std::vector<double> & effort) { hw_states_efforts_ = effort; }
  void DiagnoseEStop(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    UGVSystem::DiagnoseEStop(status);
  }

  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockRobotDriver::NiceMock>
  GetMockRobotDriver()
  {
    return mock_robot_driver_;
  }

  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockGPIOController::NiceMock>
  GetMockGPIOController()
  {
    return mock_gpio_controller_;
  }

  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockEStop::NiceMock> GetMockEStop()
  {
    return mock_e_stop_;
  }

  bool HasSystemROSInterface() const { return system_ros_interface_ != nullptr; }
  bool HasEStop() const { return e_stop_ != nullptr; }
  bool HasGPIOController() const { return gpio_controller_ != nullptr; }

  void TickCANopenResyncWatchdog(const bool data_timed_out)
  {
    UpdateCANopenResyncWatchdog(data_timed_out);
  }

  void EnableCANopenResyncWatchdog() { canopen_settings_.canopen_resync_watchdog = true; }

  const husarion_ugv_hardware_interfaces::CANopenSettings & GetCANopenSettings() const
  {
    return canopen_settings_;
  }

  std::chrono::seconds GetCANopenResyncWatchdogTimeout() const
  {
    return canopen_resync_watchdog_timeout_;
  }

  void SetPDOTimeoutOngoingSince(const std::chrono::steady_clock::time_point time_point)
  {
    pdo_timeout_ongoing_ = true;
    pdo_timeout_since_ = time_point;
  }

  using NiceMock = testing::NiceMock<MockUGVSystem>;

private:
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockRobotDriver::NiceMock>
    mock_robot_driver_;
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockGPIOController::NiceMock>
    mock_gpio_controller_;
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockEStop::NiceMock> mock_e_stop_;
};

class TestUGVSystem : public ::testing::Test
{
public:
  TestUGVSystem()
  {
    ugv_system_ =
      std::make_shared<MockUGVSystem::NiceMock>(std::vector<std::string>{"fl", "fr", "rl", "rr"});

    hardware_info_ = husarion_ugv_hardware_interfaces_test::GenerateDefaultHardwareInfo();
  }

  ~TestUGVSystem() {}

protected:
  std::shared_ptr<MockUGVSystem::NiceMock> ugv_system_;
  hardware_interface::HardwareInfo hardware_info_;
};

TEST_F(TestUGVSystem, OnInit)
{
  EXPECT_CALL(*ugv_system_, ReadCANopenSettingsDriverCANIDs()).Times(1);

  auto callback_return = ugv_system_->on_init(hardware_info_);

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestUGVSystem, OnConfigure)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));

  EXPECT_CALL(*ugv_system_, DefineRobotDriver()).Times(1);
  EXPECT_CALL(*ugv_system_, ConfigureGPIOController()).Times(1);
  EXPECT_CALL(*ugv_system_, ConfigureEStop()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), Activate()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockGPIOController(), QueryControlInterfaceIOStates()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockEStop(), ReadEStopState()).Times(1);
  auto callback_return = ugv_system_->on_configure(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, OnCleanup)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), Deinitialize()).Times(1);
  auto callback_return = ugv_system_->on_cleanup(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, OnActivate)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
  auto callback_return = ugv_system_->on_activate(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, OnDeactivate)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
  ASSERT_NO_THROW(ugv_system_->on_activate(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop()).Times(1);
  auto callback_return = ugv_system_->on_deactivate(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, OnShutdown)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), Deinitialize()).Times(1);
  auto callback_return = ugv_system_->on_shutdown(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);

  rclcpp::shutdown();
}

void ExpectTeardownOrder(MockUGVSystem & ugv_system)
{
  // The driver is deinitialized once nothing can call into it, and the GPIO controller, which cuts
  // motor power when destroyed, goes after the driver.
  testing::InSequence sequence;
  // The e-stop refuses resets before the watchdog stops, so none can turn it on again. The ROS
  // interface goes before the GPIO monitor, so no service callback reads a stopped monitor.
  EXPECT_CALL(*ugv_system.GetMockEStop(), Close()).WillOnce(testing::Invoke([&]() {
    EXPECT_TRUE(ugv_system.HasSystemROSInterface());
  }));
  EXPECT_CALL(ugv_system.GetMockGPIOController()->GetMockGPIODriver(), GPIOMonitorDisable())
    .WillOnce(testing::Invoke([&]() { EXPECT_FALSE(ugv_system.HasSystemROSInterface()); }));
  EXPECT_CALL(*ugv_system.GetMockRobotDriver(), Deinitialize()).WillOnce(testing::Invoke([&]() {
    EXPECT_FALSE(ugv_system.HasSystemROSInterface());
    EXPECT_FALSE(ugv_system.HasEStop());
    EXPECT_TRUE(ugv_system.HasGPIOController());
  }));
}

TEST_F(TestUGVSystem, OnShutdownTearsDownCallersBeforeCallees)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop());
  ExpectTeardownOrder(*ugv_system_);
  EXPECT_EQ(
    ugv_system_->on_shutdown(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(ugv_system_->HasGPIOController());

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, OnCleanupTearsDownCallersBeforeCallees)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  ExpectTeardownOrder(*ugv_system_);
  EXPECT_EQ(
    ugv_system_->on_cleanup(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(ugv_system_->HasGPIOController());

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, OnErrorTearsDownCallersBeforeCallees)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop());
  ExpectTeardownOrder(*ugv_system_);
  EXPECT_EQ(
    ugv_system_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(ugv_system_->HasGPIOController());

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, OnErrorAndShutdownAfterConfigureFailedEarly)
{
  // The driver fails to initialize, so configure returns before the e-stop exists.
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ON_CALL(*ugv_system_->GetMockRobotDriver(), Initialize())
    .WillByDefault(::testing::Throw(std::runtime_error("CAN bus not available")));
  ASSERT_EQ(
    ugv_system_->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  ASSERT_FALSE(ugv_system_->HasEStop());

  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop()).Times(0);
  EXPECT_EQ(
    ugv_system_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    ugv_system_->on_shutdown(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestUGVSystem, OnErrorTearsDownWhenTheTriggerFails)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop())
    .WillOnce(::testing::Throw(std::runtime_error("GPIO write failed")));
  ExpectTeardownOrder(*ugv_system_);
  EXPECT_EQ(
    ugv_system_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  EXPECT_FALSE(ugv_system_->HasGPIOController());

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, GPIOEdgeCallbackDuringTeardown)
{
  rclcpp::init(0, nullptr);

  // The edge callback runs on the GPIO monitor thread and on any SetPinValue() caller. One that
  // keeps firing while the component tears down must neither crash nor publish afterwards.
  std::function<void(const husarion_ugv_hardware_interfaces::GPIOInfo &)> edge_callback;
  ON_CALL(
    ugv_system_->GetMockGPIOController()->GetMockGPIODriver(),
    ConfigureEdgeEventCallback(testing::_))
    .WillByDefault(testing::SaveArg<0>(&edge_callback));
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
  ASSERT_TRUE(edge_callback);

  std::atomic_bool stop{false};
  auto firing = std::async(std::launch::async, [&]() {
    const husarion_ugv_hardware_interfaces::GPIOInfo info{
      husarion_ugv_hardware_interfaces::GPIOPin::FAN_SW, gpiod::line::direction::OUTPUT};
    while (!stop) {
      edge_callback(info);
    }
  });
  EXPECT_EQ(
    ugv_system_->on_shutdown(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  stop = true;
  firing.get();
  EXPECT_FALSE(ugv_system_->HasSystemROSInterface());

  rclcpp::shutdown();
}

TEST(TestSystemROSInterfaceDiagnostics, ThrowingTaskReportsStale)
{
  auto task = husarion_ugv_hardware_interfaces::SystemROSInterface::ReportStaleOnException(
    [](diagnostic_updater::DiagnosticStatusWrapper &) {
      throw std::runtime_error("driver deinitialized");
    });

  diagnostic_updater::DiagnosticStatusWrapper status;
  EXPECT_NO_THROW(task(status));
  EXPECT_EQ(status.level, diagnostic_updater::DiagnosticStatusWrapper::STALE);
  EXPECT_EQ(status.message, "Diagnostic task failed: driver deinitialized");
}

TEST(TestSystemROSInterfaceDiagnostics, TaskThatSucceedsIsUnchanged)
{
  auto task = husarion_ugv_hardware_interfaces::SystemROSInterface::ReportStaleOnException(
    [](diagnostic_updater::DiagnosticStatusWrapper & status) {
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "warn");
    });

  diagnostic_updater::DiagnosticStatusWrapper status;
  task(status);
  EXPECT_EQ(status.level, diagnostic_updater::DiagnosticStatusWrapper::WARN);
  EXPECT_EQ(status.message, "warn");
}

TEST_F(TestUGVSystem, OnError)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), Deinitialize()).Times(1);
  auto callback_return = ugv_system_->on_error(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, ExportStateInterfacesInitialValues)
{
  std::vector<std::string> prefixes = {"fl", "fr", "rl", "rr"};

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));

  std::vector<hardware_interface::StateInterface> state_interfaces =
    ugv_system_->export_state_interfaces();

  ASSERT_EQ(state_interfaces.size(), 12);

  int i = 0;
  for (const auto & prefix : prefixes) {
    EXPECT_EQ(
      state_interfaces[i].get_name(),
      prefix + "_wheel_joint/" + hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(
      state_interfaces[i + 1].get_name(),
      prefix + "_wheel_joint/" + hardware_interface::HW_IF_VELOCITY);
    EXPECT_EQ(
      state_interfaces[i + 2].get_name(),
      prefix + "_wheel_joint/" + hardware_interface::HW_IF_EFFORT);

    i += 3;
  }

  auto all_interfaces_are_nan = std::all_of(
    state_interfaces.begin(), state_interfaces.end(),
    [](const hardware_interface::StateInterface & state) {
      return std::isnan(state.get_optional().value());
    });

  EXPECT_TRUE(all_interfaces_are_nan);
}

TEST_F(TestUGVSystem, ExportStateInterfaces)
{
  std::vector<double> position = {1.0, 2.0, 3.0, 4.0};
  std::vector<double> velocity = {5.0, 6.0, 7.0, 8.0};
  std::vector<double> effort = {9.0, 10.0, 11.0, 12.0};

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));

  ugv_system_->SetHwStatePosition(position);
  ugv_system_->SetHwStateVelocity(velocity);
  ugv_system_->SetHwStateEffort(effort);

  std::vector<hardware_interface::StateInterface> state_interfaces =
    ugv_system_->export_state_interfaces();

  ASSERT_EQ(state_interfaces.size(), 12);

  for (std::size_t i = 0; i < 4; i++) {
    EXPECT_DOUBLE_EQ(state_interfaces[i * 3].get_optional().value(), position[i]);
    EXPECT_DOUBLE_EQ(state_interfaces[i * 3 + 1].get_optional().value(), velocity[i]);
    EXPECT_DOUBLE_EQ(state_interfaces[i * 3 + 2].get_optional().value(), effort[i]);
  }
}

TEST_F(TestUGVSystem, ExportCommandInterfacesInitialValues)
{
  std::vector<std::string> prefixes = {"fl", "fr", "rl", "rr"};

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));

  std::vector<hardware_interface::CommandInterface> command_interfaces =
    ugv_system_->export_command_interfaces();

  ASSERT_EQ(command_interfaces.size(), 4);

  EXPECT_EQ(
    command_interfaces[0].get_name(),
    std::string("fl_wheel_joint/") + hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(
    command_interfaces[1].get_name(),
    std::string("fr_wheel_joint/") + hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(
    command_interfaces[2].get_name(),
    std::string("rl_wheel_joint/") + hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(
    command_interfaces[3].get_name(),
    std::string("rr_wheel_joint/") + hardware_interface::HW_IF_VELOCITY);

  std::for_each(
    command_interfaces.begin(), command_interfaces.end(),
    [](const hardware_interface::CommandInterface & command) {
      EXPECT_DOUBLE_EQ(command.get_optional().value(), 0.0);
    });
}

TEST_F(TestUGVSystem, ExportCommandInterfaces)
{
  std::vector<double> velocity = {1.0, 2.0, 3.0, 4.0};

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));

  ugv_system_->SetHwCommandVelocity(velocity);

  std::vector<hardware_interface::CommandInterface> command_interfaces =
    ugv_system_->export_command_interfaces();

  ASSERT_EQ(command_interfaces.size(), 4);

  for (std::size_t i = 0; i < 4; i++) {
    EXPECT_DOUBLE_EQ(command_interfaces[i].get_optional().value(), velocity[i]);
  }
}

TEST_F(TestUGVSystem, Read)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
  ASSERT_NO_THROW(ugv_system_->on_activate(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), UpdateMotorsState()).Times(1);
  EXPECT_CALL(*ugv_system_, UpdateHwStates()).Times(1);
  EXPECT_CALL(*ugv_system_, UpdateMotorsStateDataTimedOut()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), UpdateDriversState()).Times(1);
  EXPECT_CALL(*ugv_system_, UpdateDriverStateMsg()).Times(1);
  EXPECT_CALL(*ugv_system_, UpdateFlagErrors()).Times(1);
  EXPECT_CALL(*ugv_system_, UpdateDriverStateDataTimedOut()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockEStop(), ReadEStopState()).Times(1);

  auto callback_return = ugv_system_->read(
    rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0));

  EXPECT_EQ(callback_return, hardware_interface::return_type::OK);

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, ReadKeepsDriverStateRate)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
  ASSERT_NO_THROW(ugv_system_->on_activate(rclcpp_lifecycle::State()));

  int driver_state_updates = 0;
  ON_CALL(*ugv_system_, UpdateDriverStateMsg()).WillByDefault([&]() { ++driver_state_updates; });

  // 100 Hz reads up to 0.4 ms late for one second, 20 Hz driver state expected
  const int64_t start_ns = 1'000'000'000;
  for (int i = 0; i < 100; ++i) {
    const int64_t jitter_ns = (i % 3) * 200'000;
    ugv_system_->read(
      rclcpp::Time(start_ns + i * 10'000'000 + jitter_ns, RCL_STEADY_TIME),
      rclcpp::Duration(0, 10'000'000));
  }

  EXPECT_EQ(driver_state_updates, 20);

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, DiagnoseEStop)
{
  using husarion_ugv_hardware_interfaces::EStopLatchSource;
  rclcpp::init(0, nullptr);

  const auto e_stop_diagnostics =
    std::make_shared<husarion_ugv_hardware_interfaces::EStopDiagnostics>();
  ON_CALL(*ugv_system_->GetMockEStop(), GetDiagnostics())
    .WillByDefault(::testing::Return(e_stop_diagnostics));

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  const auto diagnose = [this]() -> diagnostic_msgs::msg::DiagnosticStatus {
    diagnostic_updater::DiagnosticStatusWrapper status;
    ugv_system_->DiagnoseEStop(status);
    return status;
  };
  const auto value_of = husarion_ugv_hardware_interfaces_test::GetDiagnosticValue;

  // A robot parked since boot or stopped by the driver is not a fault.
  auto status = diagnose();
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::OK, status.level);
  EXPECT_EQ("latched", value_of(status, "State"));

  e_stop_diagnostics->SetSource(EStopLatchSource::SOFTWARE);
  status = diagnose();
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::OK, status.level);
  EXPECT_THAT(value_of(status, "Source"), ::testing::HasSubstr("software"));

  e_stop_diagnostics->SetSource(EStopLatchSource::RELEASED);
  status = diagnose();
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::OK, status.level);
  EXPECT_EQ("released", value_of(status, "State"));

  // Only a stop by a device outside the driver asks for attention.
  e_stop_diagnostics->SetSource(EStopLatchSource::HARDWARE);
  e_stop_diagnostics->SetResetRefused("line stuck");
  status = diagnose();
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::WARN, status.level);
  EXPECT_THAT(value_of(status, "Source"), ::testing::HasSubstr("hardware line"));
  EXPECT_EQ("line stuck", value_of(status, "Last refused reset"));

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, DiagnoseEStopWithoutState)
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  ugv_system_->DiagnoseEStop(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::STALE, status.level);
}

TEST_F(TestUGVSystem, Write)
{
  rclcpp::init(0, nullptr);

  const auto velocity = std::vector<float>{1.0, 2.0, 3.0, 4.0};

  // System is deactivated, no speed commands sent
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), ReadEStopState()).Times(1);
  EXPECT_CALL(*ugv_system_, GetSpeedCommands()).Times(0);
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), SendSpeedCommands(velocity)).Times(0);

  auto callback_return = ugv_system_->write(
    rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 0));

  EXPECT_EQ(callback_return, hardware_interface::return_type::OK);

  // System is activated, speed commands sent
  ugv_system_->set_lifecycle_state(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "test_active"));

  ASSERT_NO_THROW(ugv_system_->on_activate(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), ReadEStopState()).Times(1);
  EXPECT_CALL(*ugv_system_, GetSpeedCommands()).WillOnce(::testing::Return(velocity));
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), SendSpeedCommands(velocity)).Times(1);

  callback_return = ugv_system_->write(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 0));

  EXPECT_EQ(callback_return, hardware_interface::return_type::OK);

  // System is activated, but e-stop was triggered
  EXPECT_CALL(*ugv_system_->GetMockEStop(), ReadEStopState()).WillOnce(::testing::Return(true));
  EXPECT_CALL(*ugv_system_, GetSpeedCommands()).Times(0);
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), SendSpeedCommands(velocity)).Times(0);

  callback_return = ugv_system_->write(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 0));

  EXPECT_EQ(callback_return, hardware_interface::return_type::OK);

  rclcpp::shutdown();
}

namespace
{

husarion_ugv_msgs::msg::BatteryProtection UVPMsg(const float drive_window_s)
{
  husarion_ugv_msgs::msg::BatteryProtection msg;
  msg.state = husarion_ugv_msgs::msg::BatteryProtection::STATE_UVP;
  msg.mode = husarion_ugv_msgs::msg::BatteryProtection::MODE_STOP;
  msg.reactions_enabled = true;
  msg.drive_window_s = drive_window_s;
  msg.trip_id = 1;
  msg.trip_stamp.sec = 1000;
  return msg;
}

std::string ValueOf(const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return std::string("<missing>");
}

}  // namespace

class TestUGVSystemUVP : public TestUGVSystem
{
protected:
  void Configure(const bool uvp_enable)
  {
    if (uvp_enable) {
      hardware_info_.hardware_parameters["uvp_enable"] = "true";
    }
    ON_CALL(*ugv_system_->GetMockEStop(), GetDiagnostics())
      .WillByDefault(::testing::Return(e_stop_diagnostics_));
    // The mock e-stop latches on a trigger and stays latched until the test releases it.
    ON_CALL(*ugv_system_->GetMockEStop(), ReadEStopState())
      .WillByDefault(::testing::Invoke([this]() { return latched_.load(); }));
    ON_CALL(*ugv_system_->GetMockEStop(), TriggerEStopFromUVP())
      .WillByDefault(::testing::Invoke([this]() {
        ++trigger_attempts_;
        if (refuse_triggers_ > 0) {
          --refuse_triggers_;
          return false;
        }
        latched_ = true;
        ++triggers_;
        return true;
      }));

    ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
    ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
    ASSERT_NO_THROW(ugv_system_->on_activate(rclcpp_lifecycle::State()));

    probe_node_ = std::make_shared<rclcpp::Node>("uvp_probe");
    protection_pub_ = probe_node_->create_publisher<husarion_ugv_msgs::msg::BatteryProtection>(
      "battery/protection", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  }

  // Runs the control loop at 100 Hz until the predicate holds or the timeout passes.
  bool ReadUntil(const std::function<bool()> & pred, const std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      ugv_system_->read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 10'000'000));
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  diagnostic_msgs::msg::DiagnosticStatus DiagnoseEStop()
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    ugv_system_->DiagnoseEStop(status);
    return status;
  }

  std::shared_ptr<husarion_ugv_hardware_interfaces::EStopDiagnostics> e_stop_diagnostics_ =
    std::make_shared<husarion_ugv_hardware_interfaces::EStopDiagnostics>();
  std::atomic_bool latched_{false};
  std::atomic_int triggers_{0};
  std::atomic_int trigger_attempts_{0};
  // Triggers refused before the mock e-stop accepts one, as while a reset holds the e-stop.
  std::atomic_int refuse_triggers_{0};
  rclcpp::Node::SharedPtr probe_node_;
  rclcpp::Publisher<husarion_ugv_msgs::msg::BatteryProtection>::SharedPtr protection_pub_;
};

TEST_F(TestUGVSystemUVP, DisabledAddsNothing)
{
  rclcpp::init(0, nullptr);
  Configure(false);
  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop()).Times(0);
  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStopFromUVP()).Times(0);

  protection_pub_->publish(UVPMsg(0.1f));
  EXPECT_FALSE(ReadUntil([this]() { return triggers_ > 0; }, std::chrono::milliseconds(1000)));
  EXPECT_EQ(0u, probe_node_->count_subscribers("/battery/protection"));
  EXPECT_EQ("<missing>", ValueOf(DiagnoseEStop(), "UVP"));

  rclcpp::shutdown();
}

TEST_F(TestUGVSystemUVP, TripsOnceAndReTriggersAfterTheDriveWindow)
{
  rclcpp::init(0, nullptr);
  Configure(true);
  // The protection never releases the e-stop.
  EXPECT_CALL(*ugv_system_->GetMockEStop(), ResetEStop()).Times(0);

  protection_pub_->publish(UVPMsg(0.3f));
  ASSERT_TRUE(ReadUntil([this]() { return triggers_ > 0; }, std::chrono::milliseconds(3000)));
  EXPECT_EQ(1u, probe_node_->count_subscribers("/battery/protection"));

  // The same trip triggers once.
  EXPECT_FALSE(ReadUntil([this]() { return triggers_ > 1; }, std::chrono::milliseconds(500)));
  EXPECT_EQ("tripped, holding the e-stop", ValueOf(DiagnoseEStop(), "UVP"));

  // Released, and not through the reset service: the window opens all the same.
  latched_ = false;
  const auto released = std::chrono::steady_clock::now();
  ASSERT_TRUE(ReadUntil([this]() { return triggers_ > 1; }, std::chrono::milliseconds(2000)));
  EXPECT_GE(std::chrono::steady_clock::now() - released, std::chrono::milliseconds(290));
  EXPECT_TRUE(latched_);

  // Released again after the re-trigger: a new window.
  latched_ = false;
  ASSERT_TRUE(ReadUntil([this]() { return true; }, std::chrono::milliseconds(20)));
  EXPECT_NE("<missing>", ValueOf(DiagnoseEStop(), "UVP drive window remaining (s)"));
  ASSERT_TRUE(ReadUntil([this]() { return triggers_ > 2; }, std::chrono::milliseconds(2000)));

  rclcpp::shutdown();
}

TEST_F(TestUGVSystemUVP, ReconfigureWithALatchedTripTriggersAgain)
{
  rclcpp::init(0, nullptr);
  Configure(true);
  protection_pub_->publish(UVPMsg(60.0f));
  ASSERT_TRUE(ReadUntil([this]() { return triggers_ == 1; }, std::chrono::milliseconds(3000)));

  // No publisher across the reconfigure, so only the previous run's input could trigger.
  protection_pub_.reset();
  ASSERT_NO_THROW(ugv_system_->on_cleanup(rclcpp_lifecycle::State()));
  latched_ = false;
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
  ASSERT_NO_THROW(ugv_system_->on_activate(rclcpp_lifecycle::State()));
  EXPECT_FALSE(ReadUntil([this]() { return triggers_ > 1; }, std::chrono::milliseconds(500)));
  EXPECT_EQ("not received yet", ValueOf(DiagnoseEStop(), "UVP data"));

  // The battery driver's latched message arrives on the new subscription and stops the robot.
  protection_pub_ = probe_node_->create_publisher<husarion_ugv_msgs::msg::BatteryProtection>(
    "battery/protection", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  protection_pub_->publish(UVPMsg(60.0f));
  ASSERT_TRUE(ReadUntil([this]() { return triggers_ == 2; }, std::chrono::milliseconds(3000)));
  EXPECT_NE("<missing>", ValueOf(DiagnoseEStop(), "UVP data age (s)"));

  rclcpp::shutdown();
}

TEST_F(TestUGVSystemUVP, RefusedTriggerIsRetriedNextCycle)
{
  rclcpp::init(0, nullptr);
  refuse_triggers_ = 2;
  Configure(true);
  protection_pub_->publish(UVPMsg(60.0f));
  ASSERT_TRUE(ReadUntil([this]() { return triggers_ == 1; }, std::chrono::milliseconds(3000)));
  EXPECT_EQ(3, trigger_attempts_.load());
  EXPECT_TRUE(latched_);
  EXPECT_EQ("1", ValueOf(DiagnoseEStop(), "UVP e-stop triggers"));

  // Accepted once: no more attempts for the same trip.
  EXPECT_FALSE(
    ReadUntil([this]() { return trigger_attempts_ > 3; }, std::chrono::milliseconds(300)));

  rclcpp::shutdown();
}

TEST_F(TestUGVSystemUVP, ReactionsOffNeverTrigger)
{
  rclcpp::init(0, nullptr);
  Configure(true);
  auto msg = UVPMsg(0.1f);
  msg.reactions_enabled = false;
  protection_pub_->publish(msg);
  EXPECT_FALSE(ReadUntil([this]() { return triggers_ > 0; }, std::chrono::milliseconds(1000)));
  EXPECT_EQ("not tripped", ValueOf(DiagnoseEStop(), "UVP"));

  rclcpp::shutdown();
}

TEST_F(TestUGVSystemUVP, StaleProtectionDataIsAnError)
{
  rclcpp::init(0, nullptr);
  Configure(true);
  auto msg = UVPMsg(60.0f);
  msg.state = husarion_ugv_msgs::msg::BatteryProtection::STATE_OK;
  protection_pub_->publish(msg);
  ASSERT_TRUE(ReadUntil(
    [this]() { return ValueOf(DiagnoseEStop(), "UVP data age (s)") != "<missing>"; },
    std::chrono::milliseconds(3000)));
  EXPECT_NE(diagnostic_updater::DiagnosticStatusWrapper::ERROR, DiagnoseEStop().level);

  std::this_thread::sleep_for(std::chrono::milliseconds(3200));
  const auto status = DiagnoseEStop();
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::ERROR, status.level);
  EXPECT_THAT(status.message, ::testing::HasSubstr("stale"));

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, DiagnoseEStopLatchedByUVP)
{
  using husarion_ugv_hardware_interfaces::EStopLatchSource;
  rclcpp::init(0, nullptr);

  const auto e_stop_diagnostics =
    std::make_shared<husarion_ugv_hardware_interfaces::EStopDiagnostics>();
  ON_CALL(*ugv_system_->GetMockEStop(), GetDiagnostics())
    .WillByDefault(::testing::Return(e_stop_diagnostics));
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  e_stop_diagnostics->SetSource(EStopLatchSource::UVP);
  diagnostic_updater::DiagnosticStatusWrapper status;
  ugv_system_->DiagnoseEStop(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::WARN, status.level);
  EXPECT_THAT(status.message, ::testing::HasSubstr("undervoltage"));

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, CANopenOptInsOffByDefault)
{
  for (const auto & value : {"", "false"}) {
    auto ugv_system =
      std::make_shared<MockUGVSystem::NiceMock>(std::vector<std::string>{"fl", "fr", "rl", "rr"});
    auto hardware_info = hardware_info_;
    if (std::string(value) != "") {
      hardware_info.hardware_parameters["canopen_resync_watchdog"] = value;
      hardware_info.hardware_parameters["limit_tpdo_rates"] = value;
    }
    ASSERT_EQ(ugv_system->on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_FALSE(ugv_system->GetCANopenSettings().canopen_resync_watchdog) << value;
    EXPECT_FALSE(ugv_system->GetCANopenSettings().limit_tpdo_rates) << value;

    // Without the watchdog a long timeout keeps the process running.
    ugv_system->SetPDOTimeoutOngoingSince(std::chrono::steady_clock::now() - std::chrono::hours(1));
    ugv_system->TickCANopenResyncWatchdog(true);
  }
}

TEST_F(TestUGVSystem, CANopenOptInsOn)
{
  auto hardware_info = hardware_info_;
  hardware_info.hardware_parameters["canopen_resync_watchdog"] = "true";
  hardware_info.hardware_parameters["limit_tpdo_rates"] = "True";
  ASSERT_EQ(ugv_system_->on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ugv_system_->GetCANopenSettings().canopen_resync_watchdog);
  EXPECT_TRUE(ugv_system_->GetCANopenSettings().limit_tpdo_rates);
}

TEST_F(TestUGVSystem, CANopenResyncWatchdogRestartsOnActivate)
{
  rclcpp::init(0, nullptr);

  auto hardware_info = hardware_info_;
  hardware_info.hardware_parameters["canopen_resync_watchdog"] = "true";
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  // A timeout that was ongoing an hour ago, before a deactivate and a new activation, does not
  // count: the first timed out read after the activation starts a new count.
  ugv_system_->SetPDOTimeoutOngoingSince(std::chrono::steady_clock::now() - std::chrono::hours(1));
  ASSERT_NO_THROW(ugv_system_->on_activate(rclcpp_lifecycle::State()));
  ugv_system_->TickCANopenResyncWatchdog(true);
  ugv_system_->TickCANopenResyncWatchdog(true);

  rclcpp::shutdown();
}

void ReconfigureAndTick(MockUGVSystem & ugv_system)
{
  ugv_system.SetPDOTimeoutOngoingSince(std::chrono::steady_clock::now() - std::chrono::hours(1));
  ugv_system.on_cleanup(rclcpp_lifecycle::State());
  ugv_system.on_configure(rclcpp_lifecycle::State());
  // read() runs while INACTIVE: the first timed out read on the fresh master starts a new count.
  ugv_system.TickCANopenResyncWatchdog(true);
  ugv_system.TickCANopenResyncWatchdog(true);
  // _Exit skips the leaked mock check that exit() would fail on in the forked process.
  std::_Exit(0);
}

TEST_F(TestUGVSystem, CANopenResyncWatchdogRestartsOnReconfigure)
{
  rclcpp::init(0, nullptr);

  auto hardware_info = hardware_info_;
  hardware_info.hardware_parameters["canopen_resync_watchdog"] = "true";
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
  EXPECT_EXIT(ReconfigureAndTick(*ugv_system_), ::testing::ExitedWithCode(0), "");

  rclcpp::shutdown();
}

TEST_F(TestUGVSystem, CANopenResyncWatchdog)
{
  ugv_system_->EnableCANopenResyncWatchdog();

  // A timeout that clears resets the watchdog - the next one starts counting from zero
  ugv_system_->SetPDOTimeoutOngoingSince(std::chrono::steady_clock::now() - std::chrono::hours(1));
  ugv_system_->TickCANopenResyncWatchdog(false);
  ugv_system_->TickCANopenResyncWatchdog(true);
  ugv_system_->TickCANopenResyncWatchdog(true);

  // A continuous timeout past the threshold exits the process
  ugv_system_->SetPDOTimeoutOngoingSince(std::chrono::steady_clock::now() - std::chrono::hours(1));
  EXPECT_EXIT(
    ugv_system_->TickCANopenResyncWatchdog(true),
    ::testing::ExitedWithCode(husarion_ugv_hardware_interfaces::kCANopenResyncWatchdogExitCode),
    "");
}

TEST_F(TestUGVSystem, CANopenResyncWatchdogTimeoutParameter)
{
  auto hardware_info = hardware_info_;
  hardware_info.hardware_parameters["canopen_resync_watchdog"] = "true";
  hardware_info.hardware_parameters["canopen_resync_watchdog_timeout_s"] = "7200";
  ASSERT_EQ(ugv_system_->on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(ugv_system_->GetCANopenResyncWatchdogTimeout(), std::chrono::seconds(7200));

  // An hour of timeout is still below the configured two hours
  ugv_system_->SetPDOTimeoutOngoingSince(std::chrono::steady_clock::now() - std::chrono::hours(1));
  ugv_system_->TickCANopenResyncWatchdog(true);
}

TEST_F(TestUGVSystem, CANopenResyncWatchdogTimeoutInvalid)
{
  for (const auto & value : {"0", "-5", "45s", "soon", "99999999999"}) {
    auto ugv_system =
      std::make_shared<MockUGVSystem::NiceMock>(std::vector<std::string>{"fl", "fr", "rl", "rr"});
    auto hardware_info = hardware_info_;
    hardware_info.hardware_parameters["canopen_resync_watchdog"] = "true";
    hardware_info.hardware_parameters["canopen_resync_watchdog_timeout_s"] = value;
    ASSERT_EQ(ugv_system->on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(ugv_system->GetCANopenResyncWatchdogTimeout(), std::chrono::seconds(30)) << value;
  }
}

void ReadWithHeartbeatTimeoutAndExit(MockUGVSystem & ugv_system)
{
  ugv_system.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0));
  // The count was ended, so a timed out read right after it starts a new one.
  ugv_system.TickCANopenResyncWatchdog(true);
  // _Exit skips the leaked mock check that exit() would fail on in the forked process.
  std::_Exit(0);
}

TEST_F(TestUGVSystem, CANopenResyncWatchdogIgnoresHeartbeatTimeouts)
{
  rclcpp::init(0, nullptr);

  auto hardware_info = hardware_info_;
  hardware_info.hardware_parameters["canopen_resync_watchdog"] = "true";
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
  ASSERT_NO_THROW(ugv_system_->on_activate(rclcpp_lifecycle::State()));

  // Motor controllers without power stop their heartbeats. A restart cannot heal that, so it
  // must not end in a restart loop.
  ON_CALL(*ugv_system_->GetMockRobotDriver(), UpdateMotorsState())
    .WillByDefault(::testing::Throw(std::runtime_error("Motor controller heartbeat timeout.")));

  ugv_system_->SetPDOTimeoutOngoingSince(std::chrono::steady_clock::now() - std::chrono::hours(1));
  EXPECT_EXIT(ReadWithHeartbeatTimeoutAndExit(*ugv_system_), ::testing::ExitedWithCode(0), "");

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // The death tests run after rclcpp and the mocks started threads
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  const auto result = RUN_ALL_TESTS();
  return result;
}
