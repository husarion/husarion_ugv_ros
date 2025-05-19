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
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

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
  MOCK_METHOD(void, UpdateMotorsStateDataTimedOut, (), (override));
  MOCK_METHOD(void, UpdateDriverStateMsg, (), (override));
  MOCK_METHOD(void, UpdateFlagErrors, (), (override));
  MOCK_METHOD(void, UpdateDriverStateDataTimedOut, (), (override));

  MOCK_METHOD(std::vector<float>, GetSpeedCommands, (), (const, override));

  MOCK_METHOD(void, DiagnoseErrors, (diagnostic_updater::DiagnosticStatusWrapper &), (override));
  MOCK_METHOD(void, DiagnoseStatus, (diagnostic_updater::DiagnosticStatusWrapper &), (override));

  void DefaultDefineRobotDriver()
  {
    robot_driver_ =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockRobotDriver::NiceMock>();
  }

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
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));

  EXPECT_CALL(*ugv_system_, DefineRobotDriver()).WillOnce(::testing::Invoke([&]() {
    ugv_system_->DefaultDefineRobotDriver();
  }));
  EXPECT_CALL(*ugv_system_, ConfigureGPIOController()).Times(1);
  EXPECT_CALL(*ugv_system_, ConfigureEStop()).Times(1);
  auto callback_return = ugv_system_->on_configure(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestUGVSystem, OnCleanup)
{
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), Deinitialize()).Times(1);
  auto callback_return = ugv_system_->on_cleanup(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestUGVSystem, OnActivate)
{
  rclcpp::init(0, nullptr);

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), Activate()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockGPIOController(), QueryControlInterfaceIOStates()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockEStop(), ReadEStopState()).Times(1);
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
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), Deinitialize()).Times(1);
  auto callback_return = ugv_system_->on_shutdown(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestUGVSystem, OnError)
{
  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_->GetMockEStop(), TriggerEStop()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), Deinitialize()).Times(1);
  auto callback_return = ugv_system_->on_error(rclcpp_lifecycle::State());

  EXPECT_EQ(callback_return, hardware_interface::CallbackReturn::SUCCESS);
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

TEST_F(TestUGVSystem, Write)
{
  rclcpp::init(0, nullptr);

  const auto velocity = std::vector<float>{1.0, 2.0, 3.0, 4.0};

  ASSERT_NO_THROW(ugv_system_->on_init(hardware_info_));
  ASSERT_NO_THROW(ugv_system_->on_configure(rclcpp_lifecycle::State()));
  ASSERT_NO_THROW(ugv_system_->on_activate(rclcpp_lifecycle::State()));

  EXPECT_CALL(*ugv_system_, GetSpeedCommands()).WillOnce(::testing::Return(velocity));
  EXPECT_CALL(*ugv_system_->GetMockEStop(), ReadEStopState()).Times(1);
  EXPECT_CALL(*ugv_system_->GetMockRobotDriver(), SendSpeedCommands(velocity)).Times(1);

  auto callback_return = ugv_system_->write(
    rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 0));

  EXPECT_EQ(callback_return, hardware_interface::return_type::OK);

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  return result;
}
