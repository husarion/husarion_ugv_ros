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

#include <rclcpp/rclcpp.hpp>

#include "husarion_ugv_hardware_interfaces/robot_system/panther_system.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_robot_driver.hpp"

#include "utils/system_test_utils.hpp"
#include "utils/test_constants.hpp"

class PantherSystemWrapper : public husarion_ugv_hardware_interfaces::PantherSystem
{
public:
  PantherSystemWrapper() : PantherSystem()
  {
    mock_robot_driver =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockRobotDriver::NiceMock>();
    mock_gpio_controller =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockGPIOController::NiceMock>();
    mock_e_stop = std::make_shared<husarion_ugv_hardware_interfaces_test::MockEStop::NiceMock>();

    ON_CALL(
      *mock_robot_driver,
      GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::FRONT)))
      .WillByDefault(::testing::ReturnRef(default_driver_data));
    ON_CALL(
      *mock_robot_driver,
      GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::REAR)))
      .WillByDefault(::testing::ReturnRef(default_driver_data));
  }

  void DefineRobotDriver() override { robot_driver_ = mock_robot_driver; }
  void ConfigureGPIOController() override { gpio_controller_ = mock_gpio_controller; }
  void ConfigureEStop() override { e_stop_ = mock_e_stop; }

  void ReadCANopenSettingsDriverCANIDs() { PantherSystem::ReadCANopenSettingsDriverCANIDs(); }
  void UpdateHwStates() { PantherSystem::UpdateHwStates(); }
  void UpdateMotorsStateDataTimedOut() { PantherSystem::UpdateMotorsStateDataTimedOut(); }
  void UpdateDriverStateDataTimedOut() { PantherSystem::UpdateDriverStateDataTimedOut(); }

  void UpdateDriverStateMsg() { PantherSystem::UpdateDriverStateMsg(); }
  void UpdateFlagErrors() { PantherSystem::UpdateFlagErrors(); }
  std::vector<float> GetSpeedCommands() const { return PantherSystem::GetSpeedCommands(); }

  void SetHwCommandsVelocities(std::vector<double> & velocities)
  {
    hw_commands_velocities_[0] = velocities[0];
    hw_commands_velocities_[1] = velocities[1];
    hw_commands_velocities_[2] = velocities[2];
    hw_commands_velocities_[3] = velocities[3];
  }

  husarion_ugv_hardware_interfaces::CANopenSettings GetCANopenSettings()
  {
    return canopen_settings_;
  }
  std::vector<double> GetHwStatesPositions() { return hw_states_positions_; }
  std::vector<double> GetHwStatesVelocities() { return hw_states_velocities_; }
  std::vector<double> GetHwStatesEfforts() { return hw_states_efforts_; }

  std::shared_ptr<husarion_ugv_hardware_interfaces::RoboteqErrorFilter> GetRoboteqErrorFilter()
  {
    return roboteq_error_filter_;
  }

  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockRobotDriver::NiceMock>
    mock_robot_driver;
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockGPIOController::NiceMock>
    mock_gpio_controller;
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockEStop::NiceMock> mock_e_stop;

private:
  husarion_ugv_hardware_interfaces::DriverData default_driver_data =
    husarion_ugv_hardware_interfaces::DriverData(
      husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
};

class TestPantherSystem : public ::testing::Test
{
public:
  TestPantherSystem()
  {
    panther_system_ = std::make_shared<PantherSystemWrapper>();

    hardware_info_ = husarion_ugv_hardware_interfaces_test::GenerateDefaultHardwareInfo();
    hardware_info_.hardware_parameters.emplace("front_driver_can_id", "1");
    hardware_info_.hardware_parameters.emplace("rear_driver_can_id", "2");

    panther_system_->on_init(hardware_info_);
    panther_system_->on_configure(rclcpp_lifecycle::State());
  }

  ~TestPantherSystem() {}

protected:
  std::shared_ptr<PantherSystemWrapper> panther_system_;
  hardware_interface::HardwareInfo hardware_info_;
};

TEST_F(TestPantherSystem, ReadCANopenSettingsDriverCANIDs)
{
  ASSERT_NO_THROW(panther_system_->ReadCANopenSettingsDriverCANIDs());

  const auto canopen_settings = panther_system_->GetCANopenSettings();

  EXPECT_EQ(canopen_settings.driver_can_ids.size(), 2);
  EXPECT_EQ(
    canopen_settings.driver_can_ids.at(husarion_ugv_hardware_interfaces::DriverNames::FRONT), 1);
  EXPECT_EQ(
    canopen_settings.driver_can_ids.at(husarion_ugv_hardware_interfaces::DriverNames::REAR), 2);
}

TEST_F(TestPantherSystem, UpdateHwStates)
{
  const std::int32_t fl_pos = 10;
  const std::int16_t fl_vel = 20;
  const std::int16_t fl_eff = 30;
  const std::int32_t fr_pos = 40;
  const std::int16_t fr_vel = 50;
  const std::int16_t fr_eff = 60;
  const std::int32_t rl_pos = 70;
  const std::int16_t rl_vel = 80;
  const std::int16_t rl_eff = 90;
  const std::int32_t rr_pos = 100;
  const std::int16_t rr_vel = 110;
  const std::int16_t rr_eff = 120;

  const auto fl_expected_pos = fl_pos * husarion_ugv_hardware_interfaces_test::kRbtqPosFbToRad;
  const auto fl_expected_vel = fl_vel *
                               husarion_ugv_hardware_interfaces_test::kRbtqVelFbToRadPerSec;
  const auto fl_expected_eff = fl_eff *
                               husarion_ugv_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  const auto fr_expected_pos = fr_pos * husarion_ugv_hardware_interfaces_test::kRbtqPosFbToRad;
  const auto fr_expected_vel = fr_vel *
                               husarion_ugv_hardware_interfaces_test::kRbtqVelFbToRadPerSec;
  const auto fr_expected_eff = fr_eff *
                               husarion_ugv_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  const auto rl_expected_pos = rl_pos * husarion_ugv_hardware_interfaces_test::kRbtqPosFbToRad;
  const auto rl_expected_vel = rl_vel *
                               husarion_ugv_hardware_interfaces_test::kRbtqVelFbToRadPerSec;
  const auto rl_expected_eff = rl_eff *
                               husarion_ugv_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  const auto rr_expected_pos = rr_pos * husarion_ugv_hardware_interfaces_test::kRbtqPosFbToRad;
  const auto rr_expected_vel = rr_vel *
                               husarion_ugv_hardware_interfaces_test::kRbtqVelFbToRadPerSec;
  const auto rr_expected_eff = rr_eff *
                               husarion_ugv_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;

  husarion_ugv_hardware_interfaces::MotorDriverState fl_motor_driver_state = {
    fl_pos, fl_vel, fl_eff, {0, 0}, {0, 0}};
  husarion_ugv_hardware_interfaces::MotorDriverState fr_motor_driver_state = {
    fr_pos, fr_vel, fr_eff, {0, 0}, {0, 0}};
  husarion_ugv_hardware_interfaces::MotorDriverState rl_motor_driver_state = {
    rl_pos, rl_vel, rl_eff, {0, 0}, {0, 0}};
  husarion_ugv_hardware_interfaces::MotorDriverState rr_motor_driver_state = {
    rr_pos, rr_vel, rr_eff, {0, 0}, {0, 0}};

  husarion_ugv_hardware_interfaces::DriverData front_roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  husarion_ugv_hardware_interfaces::DriverData rear_roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  // left - channel 2, right - channel 1
  front_roboteq_data.SetMotorsStates(fr_motor_driver_state, fl_motor_driver_state, false);
  rear_roboteq_data.SetMotorsStates(rr_motor_driver_state, rl_motor_driver_state, false);

  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::FRONT)))
    .WillOnce(::testing::ReturnRef(front_roboteq_data));
  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::REAR)))
    .WillOnce(::testing::ReturnRef(rear_roboteq_data));

  ASSERT_NO_THROW(panther_system_->UpdateHwStates());

  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesPositions()[0], fl_expected_pos);
  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesPositions()[1], fr_expected_pos);
  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesPositions()[2], rl_expected_pos);
  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesPositions()[3], rr_expected_pos);

  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesVelocities()[0], fl_expected_vel);
  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesVelocities()[1], fr_expected_vel);
  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesVelocities()[2], rl_expected_vel);
  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesVelocities()[3], rr_expected_vel);

  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesEfforts()[0], fl_expected_eff);
  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesEfforts()[1], fr_expected_eff);
  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesEfforts()[2], rl_expected_eff);
  EXPECT_FLOAT_EQ(panther_system_->GetHwStatesEfforts()[3], rr_expected_eff);
}

TEST_F(TestPantherSystem, UpdateMotorsStateDataTimedOut)
{
  husarion_ugv_hardware_interfaces::MotorDriverState motor_driver_state;

  husarion_ugv_hardware_interfaces::DriverData roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  roboteq_data.SetMotorsStates(motor_driver_state, motor_driver_state, true);

  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::FRONT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));
  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::REAR)))
    .Times(0);

  panther_system_->UpdateMotorsStateDataTimedOut();

  auto error_map = panther_system_->GetRoboteqErrorFilter()->GetErrorMap();
  auto error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::READ_PDO_MOTOR_STATES));

  EXPECT_TRUE(error);

  // check if reset error works
  roboteq_data.SetMotorsStates(motor_driver_state, motor_driver_state, false);

  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::FRONT)))
    .Times(1);
  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::REAR)))
    .Times(1);

  panther_system_->GetRoboteqErrorFilter()->SetClearErrorsFlag();
  panther_system_->UpdateMotorsStateDataTimedOut();

  error_map = panther_system_->GetRoboteqErrorFilter()->GetErrorMap();
  error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::READ_PDO_MOTOR_STATES));

  EXPECT_FALSE(error);
}

TEST_F(TestPantherSystem, UpdateDriverStateMsg)
{
  // TODO requires subscribing to DriverState topic. Implement in the future or add abstraction for
  // system_ros_interface_
}

TEST_F(TestPantherSystem, UpdateFlagErrors)
{
  husarion_ugv_hardware_interfaces::DriverState driver_state;
  driver_state.fault_flags = 0b01;
  driver_state.script_flags = 0;
  driver_state.runtime_stat_flag_channel_1 = 0;
  driver_state.runtime_stat_flag_channel_2 = 0;

  husarion_ugv_hardware_interfaces::DriverData roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  roboteq_data.SetDriverState(driver_state, false);

  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::FRONT)))
    .Times(1);
  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::REAR)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  panther_system_->UpdateFlagErrors();

  auto error_map = panther_system_->GetRoboteqErrorFilter()->GetErrorMap();
  auto error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::ROBOTEQ_DRIVER));

  EXPECT_TRUE(error);

  // check if reset error works
  driver_state.fault_flags = 0;
  roboteq_data.SetDriverState(driver_state, false);

  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::FRONT)))
    .Times(1);
  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::REAR)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  panther_system_->GetRoboteqErrorFilter()->SetClearErrorsFlag();
  panther_system_->UpdateFlagErrors();

  error_map = panther_system_->GetRoboteqErrorFilter()->GetErrorMap();
  error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::ROBOTEQ_DRIVER));

  EXPECT_FALSE(error);
}

TEST_F(TestPantherSystem, UpdateDriverStateDataTimedOut)
{
  husarion_ugv_hardware_interfaces::DriverState driver_state;

  husarion_ugv_hardware_interfaces::DriverData roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  roboteq_data.SetDriverState(driver_state, true);

  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::FRONT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));
  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::REAR)))
    .Times(0);

  panther_system_->UpdateDriverStateDataTimedOut();

  auto error_map = panther_system_->GetRoboteqErrorFilter()->GetErrorMap();
  auto error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::READ_PDO_DRIVER_STATE));

  EXPECT_TRUE(error);

  // check if reset error works
  roboteq_data.SetDriverState(driver_state, false);

  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::FRONT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));
  EXPECT_CALL(
    *panther_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::REAR)))
    .Times(1);

  panther_system_->GetRoboteqErrorFilter()->SetClearErrorsFlag();
  panther_system_->UpdateDriverStateDataTimedOut();

  error_map = panther_system_->GetRoboteqErrorFilter()->GetErrorMap();
  error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::READ_PDO_DRIVER_STATE));

  EXPECT_FALSE(error);
}

TEST_F(TestPantherSystem, GetSpeedCommands)
{
  const auto fl_v = 0.1;
  const auto fr_v = 0.2;
  const auto rl_v = 0.3;
  const auto rr_v = 0.4;

  std::vector<double> velocities = {fl_v, fr_v, rl_v, rr_v};
  panther_system_->SetHwCommandsVelocities(velocities);
  const auto speed_cmd = panther_system_->GetSpeedCommands();

  ASSERT_EQ(speed_cmd.size(), 4);
  EXPECT_FLOAT_EQ(speed_cmd[0], fl_v);
  EXPECT_FLOAT_EQ(speed_cmd[1], fr_v);
  EXPECT_FLOAT_EQ(speed_cmd[2], rl_v);
  EXPECT_FLOAT_EQ(speed_cmd[3], rr_v);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  return result;
}
