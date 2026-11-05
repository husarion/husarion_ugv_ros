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

#include <sys/sysinfo.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "husarion_ugv_hardware_interfaces/robot_system/lynx_system.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_robot_driver.hpp"

#include "utils/system_test_utils.hpp"
#include "utils/test_constants.hpp"

class LynxSystemWrapper : public husarion_ugv_hardware_interfaces::LynxSystem
{
public:
  LynxSystemWrapper() : LynxSystem()
  {
    mock_robot_driver =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockRobotDriver::NiceMock>();
    mock_gpio_controller =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockGPIOController::NiceMock>();
    mock_e_stop = std::make_shared<husarion_ugv_hardware_interfaces_test::MockEStop::NiceMock>();

    e_stop_diagnostics = std::make_shared<husarion_ugv_hardware_interfaces::EStopDiagnostics>();
    e_stop_diagnostics->SetSource(husarion_ugv_hardware_interfaces::EStopLatchSource::RELEASED);
    ON_CALL(*mock_e_stop, GetDiagnostics()).WillByDefault(::testing::Return(e_stop_diagnostics));
  }

  void DefineRobotDriver() override { robot_driver_ = mock_robot_driver; }
  void ConfigureGPIOController() override { gpio_controller_ = mock_gpio_controller; }
  void ConfigureEStop() override { e_stop_ = mock_e_stop; }

  void ReadCANopenSettingsDriverCANIDs() { LynxSystem::ReadCANopenSettingsDriverCANIDs(); }
  void UpdateHwStates() { LynxSystem::UpdateHwStates(); }
  bool UpdateMotorsStateDataTimedOut() { return LynxSystem::UpdateMotorsStateDataTimedOut(); }
  void UpdateDriverStateDataTimedOut() { LynxSystem::UpdateDriverStateDataTimedOut(); }

  void UpdateDriverStateMsg() { LynxSystem::UpdateDriverStateMsg(); }
  void UpdateFlagErrors() { LynxSystem::UpdateFlagErrors(); }
  void MotorTorqueEnable(const bool enable) { LynxSystem::MotorTorqueEnable(enable); }
  void TickCANopenResyncWatchdog(const bool data_timed_out)
  {
    UpdateCANopenResyncWatchdog(data_timed_out);
  }
  void DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    LynxSystem::DiagnoseErrors(status);
  }
  void DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    LynxSystem::DiagnoseStatus(status);
  }
  std::vector<float> GetSpeedCommands() const { return LynxSystem::GetSpeedCommands(); }

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
  unsigned GetGPIOWatchdogSchedPriority() const { return gpio_watchdog_sched_priority_; }
  unsigned GetGPIOMonitorSchedPriority() const { return gpio_monitor_sched_priority_; }
  int GetGPIOWatchdogCPU() const { return gpio_watchdog_cpu_; }
  int GetGPIOMonitorCPU() const { return gpio_monitor_cpu_; }
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
  std::shared_ptr<husarion_ugv_hardware_interfaces::EStopDiagnostics> e_stop_diagnostics;
};

class TestLynxSystem : public ::testing::Test
{
public:
  TestLynxSystem()
  {
    rclcpp::init(0, nullptr);

    lynx_system_ = std::make_shared<LynxSystemWrapper>();

    hardware_info_ = husarion_ugv_hardware_interfaces_test::GenerateDefaultHardwareInfo();
    hardware_info_.hardware_parameters.emplace("driver_can_id", "1");

    lynx_system_->on_init(hardware_info_);
    lynx_system_->on_configure(rclcpp_lifecycle::State());
  }

  ~TestLynxSystem() { rclcpp::shutdown(); }

protected:
  std::shared_ptr<LynxSystemWrapper> lynx_system_;
  hardware_interface::HardwareInfo hardware_info_;
};

TEST_F(TestLynxSystem, ReadCANopenSettingsDriverCANIDs)
{
  ASSERT_NO_THROW(lynx_system_->ReadCANopenSettingsDriverCANIDs());

  const auto canopen_settings = lynx_system_->GetCANopenSettings();

  EXPECT_EQ(canopen_settings.driver_can_ids.size(), 1);
  EXPECT_EQ(
    canopen_settings.driver_can_ids.at(husarion_ugv_hardware_interfaces::DriverNames::DEFAULT), 1);
}

TEST_F(TestLynxSystem, RTThreadPrioritiesDefault)
{
  const auto canopen_settings = lynx_system_->GetCANopenSettings();

  EXPECT_EQ(canopen_settings.canopen_thread_sched_priority, 70u);
  EXPECT_EQ(canopen_settings.rpdo_dispatch_sched_priority, 65u);
  EXPECT_EQ(lynx_system_->GetGPIOWatchdogSchedPriority(), 60u);
  EXPECT_EQ(lynx_system_->GetGPIOMonitorSchedPriority(), 60u);
}

TEST_F(TestLynxSystem, RTThreadPrioritiesFromParameters)
{
  auto hardware_info = hardware_info_;
  hardware_info.hardware_parameters["canopen_thread_priority"] = "80";
  hardware_info.hardware_parameters["rpdo_dispatch_priority"] = "75";
  hardware_info.hardware_parameters["gpio_watchdog_priority"] = "62";
  hardware_info.hardware_parameters["gpio_monitor_priority"] = "55";

  auto lynx_system = std::make_shared<LynxSystemWrapper>();
  ASSERT_EQ(lynx_system->on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  const auto canopen_settings = lynx_system->GetCANopenSettings();
  EXPECT_EQ(canopen_settings.canopen_thread_sched_priority, 80u);
  EXPECT_EQ(canopen_settings.rpdo_dispatch_sched_priority, 75u);
  EXPECT_EQ(lynx_system->GetGPIOWatchdogSchedPriority(), 62u);
  EXPECT_EQ(lynx_system->GetGPIOMonitorSchedPriority(), 55u);
}

TEST_F(TestLynxSystem, RTThreadPrioritiesInvalidFallBackToDefault)
{
  auto hardware_info = hardware_info_;
  hardware_info.hardware_parameters["canopen_thread_priority"] = "99";
  hardware_info.hardware_parameters["rpdo_dispatch_priority"] = "0";
  hardware_info.hardware_parameters["gpio_watchdog_priority"] = "-1";
  hardware_info.hardware_parameters["gpio_monitor_priority"] = "high";

  auto lynx_system = std::make_shared<LynxSystemWrapper>();
  ASSERT_EQ(lynx_system->on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  const auto canopen_settings = lynx_system->GetCANopenSettings();
  EXPECT_EQ(canopen_settings.canopen_thread_sched_priority, 70u);
  EXPECT_EQ(canopen_settings.rpdo_dispatch_sched_priority, 65u);
  EXPECT_EQ(lynx_system->GetGPIOWatchdogSchedPriority(), 60u);
  EXPECT_EQ(lynx_system->GetGPIOMonitorSchedPriority(), 60u);
}

TEST_F(TestLynxSystem, RTThreadCPUsDefault)
{
  // The defaults are the robot's layout and only valid on a machine with at least four CPUs.
  const int control_cpu = get_nprocs_conf() > 3 ? 3 : -1;
  const auto canopen_settings = lynx_system_->GetCANopenSettings();

  EXPECT_EQ(canopen_settings.canopen_thread_cpu, 0);
  EXPECT_EQ(canopen_settings.rpdo_dispatch_cpu, 0);
  EXPECT_EQ(lynx_system_->GetGPIOWatchdogCPU(), control_cpu);
  EXPECT_EQ(lynx_system_->GetGPIOMonitorCPU(), control_cpu);
}

TEST_F(TestLynxSystem, RTThreadCPUsFromParameters)
{
  auto hardware_info = hardware_info_;
  hardware_info.hardware_parameters["canopen_thread_cpu"] = "1";
  hardware_info.hardware_parameters["rpdo_dispatch_cpu"] = "-1";
  hardware_info.hardware_parameters["gpio_watchdog_cpu"] = "0";
  hardware_info.hardware_parameters["gpio_monitor_cpu"] = "-1";

  auto lynx_system = std::make_shared<LynxSystemWrapper>();
  ASSERT_EQ(lynx_system->on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  const auto canopen_settings = lynx_system->GetCANopenSettings();
  EXPECT_EQ(canopen_settings.canopen_thread_cpu, get_nprocs_conf() > 1 ? 1 : -1);
  EXPECT_EQ(canopen_settings.rpdo_dispatch_cpu, -1);
  EXPECT_EQ(lynx_system->GetGPIOWatchdogCPU(), 0);
  EXPECT_EQ(lynx_system->GetGPIOMonitorCPU(), -1);
}

TEST_F(TestLynxSystem, RTThreadCPUsInvalidAreNotPinned)
{
  auto hardware_info = hardware_info_;
  hardware_info.hardware_parameters["canopen_thread_cpu"] = std::to_string(get_nprocs_conf());
  hardware_info.hardware_parameters["rpdo_dispatch_cpu"] = "-2";
  hardware_info.hardware_parameters["gpio_watchdog_cpu"] = "two";
  hardware_info.hardware_parameters["gpio_monitor_cpu"] = "";

  auto lynx_system = std::make_shared<LynxSystemWrapper>();
  ASSERT_EQ(lynx_system->on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  const auto canopen_settings = lynx_system->GetCANopenSettings();
  EXPECT_EQ(canopen_settings.canopen_thread_cpu, -1);
  EXPECT_EQ(canopen_settings.rpdo_dispatch_cpu, -1);
  EXPECT_EQ(lynx_system->GetGPIOWatchdogCPU(), -1);
  EXPECT_EQ(lynx_system->GetGPIOMonitorCPU(), -1);
}

TEST_F(TestLynxSystem, UpdateHwStates)
{
  const std::int32_t left_pos = 10;
  const std::int16_t left_vel = 20;
  const std::int16_t left_eff = 30;
  const std::int32_t right_pos = 40;
  const std::int16_t right_vel = 50;
  const std::int16_t right_eff = 60;

  const auto left_expected_pos = left_pos * husarion_ugv_hardware_interfaces_test::kRbtqPosFbToRad;
  const auto left_expected_vel = left_vel *
                                 husarion_ugv_hardware_interfaces_test::kRbtqVelFbToRadPerSec;
  const auto left_expected_eff =
    left_eff * husarion_ugv_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  const auto right_expected_pos = right_pos *
                                  husarion_ugv_hardware_interfaces_test::kRbtqPosFbToRad;
  const auto right_expected_vel = right_vel *
                                  husarion_ugv_hardware_interfaces_test::kRbtqVelFbToRadPerSec;
  const auto right_expected_eff =
    right_eff * husarion_ugv_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;

  husarion_ugv_hardware_interfaces::MotorDriverState left_motor_driver_state = {
    left_pos, left_vel, left_eff, {0, 0}, {0, 0}};
  husarion_ugv_hardware_interfaces::MotorDriverState right_motor_driver_state = {
    right_pos, right_vel, right_eff, {0, 0}, {0, 0}};

  husarion_ugv_hardware_interfaces::DriverData roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  // left - channel 2, right - channel 1
  roboteq_data.SetMotorsStates(right_motor_driver_state, left_motor_driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  ASSERT_NO_THROW(lynx_system_->UpdateHwStates());

  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesPositions()[0], left_expected_pos);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesPositions()[1], right_expected_pos);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesPositions()[2], left_expected_pos);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesPositions()[3], right_expected_pos);

  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesVelocities()[0], left_expected_vel);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesVelocities()[1], right_expected_vel);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesVelocities()[2], left_expected_vel);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesVelocities()[3], right_expected_vel);

  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesEfforts()[0], left_expected_eff);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesEfforts()[1], right_expected_eff);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesEfforts()[2], left_expected_eff);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesEfforts()[3], right_expected_eff);
}

TEST_F(TestLynxSystem, UpdateMotorsStateDataTimedOut)
{
  husarion_ugv_hardware_interfaces::MotorDriverState motor_driver_state;

  husarion_ugv_hardware_interfaces::DriverData roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  roboteq_data.SetMotorsStates(motor_driver_state, motor_driver_state, true);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->UpdateMotorsStateDataTimedOut();

  auto error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  auto error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::READ_PDO_MOTOR_STATES));

  EXPECT_TRUE(error);

  // check if reset error works
  roboteq_data.SetMotorsStates(motor_driver_state, motor_driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->GetRoboteqErrorFilter()->SetClearErrorsFlag();
  lynx_system_->UpdateMotorsStateDataTimedOut();

  error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::READ_PDO_MOTOR_STATES));

  EXPECT_FALSE(error);
}

TEST_F(TestLynxSystem, UpdateDriverStateMsg)
{
  // TODO requires subscribing to DriverState topic. Implement in the future or add abstraction for
  // system_ros_interface_
}

TEST_F(TestLynxSystem, UpdateFlagErrors)
{
  auto driver_state = husarion_ugv_hardware_interfaces::DriverState();
  driver_state.fault_flags = 0b01;

  husarion_ugv_hardware_interfaces::DriverData roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  roboteq_data.SetDriverState(driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->UpdateFlagErrors();

  auto error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  auto error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::ROBOTEQ_DRIVER));

  EXPECT_TRUE(error);

  // check if reset error works
  driver_state.fault_flags = 0;
  roboteq_data.SetDriverState(driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->GetRoboteqErrorFilter()->SetClearErrorsFlag();
  lynx_system_->UpdateFlagErrors();

  error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::ROBOTEQ_DRIVER));

  EXPECT_FALSE(error);
}

TEST_F(TestLynxSystem, UpdateDriverStateDataTimedOut)
{
  husarion_ugv_hardware_interfaces::DriverState driver_state;

  husarion_ugv_hardware_interfaces::DriverData roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  roboteq_data.SetDriverState(driver_state, true);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->UpdateDriverStateDataTimedOut();

  auto error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  auto error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::READ_PDO_DRIVER_STATE));

  EXPECT_TRUE(error);

  // check if reset error works
  roboteq_data.SetDriverState(driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(husarion_ugv_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->GetRoboteqErrorFilter()->SetClearErrorsFlag();
  lynx_system_->UpdateDriverStateDataTimedOut();

  error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  error = error_map.at(husarion_ugv_hardware_interfaces::error_filter_id_names.at(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::READ_PDO_DRIVER_STATE));

  EXPECT_FALSE(error);
}

TEST_F(TestLynxSystem, GetSpeedCommands)
{
  const auto fl_v = 0.1;
  const auto fr_v = 0.2;
  const auto rl_v = 0.3;
  const auto rr_v = 0.4;

  std::vector<double> velocities = {fl_v, fr_v, rl_v, rr_v};
  lynx_system_->SetHwCommandsVelocities(velocities);
  const auto speed_cmd = lynx_system_->GetSpeedCommands();

  // only front left and front right motors are used for speed commands
  ASSERT_EQ(speed_cmd.size(), 2);
  EXPECT_FLOAT_EQ(speed_cmd[0], fl_v);
  EXPECT_FLOAT_EQ(speed_cmd[1], fr_v);
}

TEST_F(TestLynxSystem, DiagnoseStatus)
{
  using husarion_ugv_hardware_interfaces_test::GetDiagnosticValue;
  using husarion_ugv_hardware_interfaces_test::MakeDriverState;

  husarion_ugv_hardware_interfaces::DriverData data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  // left - channel 2, right - channel 1
  data.SetMotorsStates({0, 0, 25, {0, 0}, {0, 0}}, {0, 0, -40, {0, 0}, {0, 0}}, false);
  data.SetDriverState(MakeDriverState(40), false);
  ON_CALL(*lynx_system_->mock_robot_driver, GetData(::testing::_))
    .WillByDefault(::testing::ReturnRef(data));

  diagnostic_updater::DiagnosticStatusWrapper status;
  lynx_system_->DiagnoseStatus(status);

  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::OK, status.level);
  EXPECT_EQ("Lynx system status monitoring.", status.message);
  EXPECT_FLOAT_EQ(1.5, std::stof(GetDiagnosticValue(status, "Driver battery current (A)")));
  EXPECT_FLOAT_EQ(40.0, std::stof(GetDiagnosticValue(status, "Driver voltage (V)")));
  EXPECT_FLOAT_EQ(2.5, std::stof(GetDiagnosticValue(status, "Motor current right (A)")));
  EXPECT_FLOAT_EQ(-4.0, std::stof(GetDiagnosticValue(status, "Motor current left (A)")));

  // The shipped Roboteq profile stops the motors at 85 C.
  data.SetDriverState(MakeDriverState(75), false);
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseStatus(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::WARN, status.level);
}

TEST_F(TestLynxSystem, DiagnoseErrorsSingleLatePDOIsAWarning)
{
  using husarion_ugv_hardware_interfaces_test::GetDiagnosticValue;

  husarion_ugv_hardware_interfaces::DriverData data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  husarion_ugv_hardware_interfaces::MotorDriverState motor_state{};
  data.SetMotorsStates(motor_state, motor_state, true);
  ON_CALL(*lynx_system_->mock_robot_driver, GetData(::testing::_))
    .WillByDefault(::testing::ReturnRef(data));

  lynx_system_->TickCANopenResyncWatchdog(true);

  diagnostic_updater::DiagnosticStatusWrapper status;
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::WARN, status.level);
  EXPECT_EQ("True", GetDiagnosticValue(status, "Driver warning: motor_states_data_timed_out"));
  EXPECT_EQ("1", GetDiagnosticValue(status, "PDO timeouts since the last report"));

  // The count is per report, and fresh data clears the warning.
  data.SetMotorsStates(motor_state, motor_state, false);
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::OK, status.level);
  EXPECT_EQ("<missing>", GetDiagnosticValue(status, "PDO timeouts since the last report"));
}

TEST_F(TestLynxSystem, DiagnoseErrorsDriverFaultIsAnError)
{
  using husarion_ugv_hardware_interfaces_test::GetDiagnosticValue;
  using husarion_ugv_hardware_interfaces_test::MakeDriverState;

  husarion_ugv_hardware_interfaces::DriverData data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  // fault flag bit 0 - overheat
  data.SetDriverState(MakeDriverState(40, 0b00000001), false);
  ON_CALL(*lynx_system_->mock_robot_driver, GetData(::testing::_))
    .WillByDefault(::testing::ReturnRef(data));

  diagnostic_updater::DiagnosticStatusWrapper status;
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::ERROR, status.level);
  EXPECT_EQ("True", GetDiagnosticValue(status, "Driver error: fault_flag.overheat"));

  lynx_system_->GetRoboteqErrorFilter()->UpdateError(
    husarion_ugv_hardware_interfaces::ErrorsFilterIds::ROBOTEQ_DRIVER, true);
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ("True", GetDiagnosticValue(status, "Motor controller fault"));
  EXPECT_EQ(
    "E-stop reset or motor torque toggle", GetDiagnosticValue(status, "Latched errors clear on"));

  data.SetDriverState(MakeDriverState(40), false);
  data.SetCANError(true);
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::ERROR, status.level);
  EXPECT_EQ("True", GetDiagnosticValue(status, "Driver error: can_error"));
}

TEST_F(TestLynxSystem, DiagnoseErrorsCurrentLimitIsAWarning)
{
  using husarion_ugv_hardware_interfaces_test::GetDiagnosticValue;
  using husarion_ugv_hardware_interfaces_test::MakeDriverState;

  husarion_ugv_hardware_interfaces::DriverData data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  // runtime flag bit 0 - amps_limit_active, channel 1 is the right motor
  data.SetDriverState(MakeDriverState(40, 0, 0b00000001), false);
  ON_CALL(*lynx_system_->mock_robot_driver, GetData(::testing::_))
    .WillByDefault(::testing::ReturnRef(data));

  diagnostic_updater::DiagnosticStatusWrapper status;
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::WARN, status.level);
  EXPECT_EQ("current limit active", GetDiagnosticValue(status, "Driver warning: right motor"));

  // With a real fault next to it the current limit is still not listed as an error.
  data.SetDriverState(MakeDriverState(40, 0b00000001, 0b00000001), false);
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::ERROR, status.level);
  EXPECT_EQ(
    "<missing>",
    GetDiagnosticValue(status, "Driver error: channel_1_motor.runtime_error.amps_limit_active"));
  EXPECT_EQ("current limit active", GetDiagnosticValue(status, "Driver warning: right motor"));
}

TEST_F(TestLynxSystem, DiagnoseErrorsSafetyStopHeldByTheEStopIsOK)
{
  using husarion_ugv_hardware_interfaces::EStopLatchSource;
  using husarion_ugv_hardware_interfaces_test::GetDiagnosticValue;
  using husarion_ugv_hardware_interfaces_test::MakeDriverState;
  constexpr std::uint8_t kSafetyStopActive = 0b00001000;

  husarion_ugv_hardware_interfaces::DriverData data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  data.SetDriverState(MakeDriverState(40, 0, kSafetyStopActive), false);
  ON_CALL(*lynx_system_->mock_robot_driver, GetData(::testing::_))
    .WillByDefault(::testing::ReturnRef(data));

  // A parked robot: the latched e-stop holds the motors through the safety stop.
  lynx_system_->e_stop_diagnostics->SetSource(EStopLatchSource::STARTUP);
  diagnostic_updater::DiagnosticStatusWrapper status;
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::OK, status.level);
  EXPECT_EQ("safety stop (held by the e-stop)", GetDiagnosticValue(status, "Driver: right motor"));
  EXPECT_EQ("<missing>", GetDiagnosticValue(status, "Driver warning: right motor"));

  // A current limit next to it is still a warning.
  data.SetDriverState(MakeDriverState(40, 0, kSafetyStopActive | 0b00000001), false);
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::WARN, status.level);
  EXPECT_EQ("current limit active", GetDiagnosticValue(status, "Driver warning: right motor"));

  // With the e-stop released nothing should hold the motors.
  data.SetDriverState(MakeDriverState(40, 0, kSafetyStopActive), false);
  lynx_system_->e_stop_diagnostics->SetSource(EStopLatchSource::RELEASED);
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::WARN, status.level);
  EXPECT_EQ("safety stop active", GetDiagnosticValue(status, "Driver warning: right motor"));
  EXPECT_EQ("<missing>", GetDiagnosticValue(status, "Driver: right motor"));
}

TEST_F(TestLynxSystem, DiagnoseErrorsTorqueDisabledByRequestIsAWarning)
{
  using husarion_ugv_hardware_interfaces_test::GetDiagnosticValue;
  using husarion_ugv_hardware_interfaces_test::MakeDriverState;
  constexpr std::uint8_t kEmergencyStopFault = 0b00010000;

  husarion_ugv_hardware_interfaces::DriverData data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  data.SetDriverState(MakeDriverState(40, kEmergencyStopFault), false);
  ON_CALL(*lynx_system_->mock_robot_driver, GetData(::testing::_))
    .WillByDefault(::testing::ReturnRef(data));
  ON_CALL(*lynx_system_->mock_e_stop, ReadEStopState()).WillByDefault(::testing::Return(true));

  // Without a torque disable request the same fault is an error.
  diagnostic_updater::DiagnosticStatusWrapper status;
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::ERROR, status.level);

  lynx_system_->MotorTorqueEnable(false);
  lynx_system_->UpdateFlagErrors();

  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::WARN, status.level);
  EXPECT_EQ("disabled by request", GetDiagnosticValue(status, "Driver warning: motor torque"));

  // Any other fault next to it is still an error.
  data.SetDriverState(MakeDriverState(40, kEmergencyStopFault | 0b00000001), false);
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::ERROR, status.level);

  lynx_system_->MotorTorqueEnable(true);
  data.SetDriverState(MakeDriverState(40, kEmergencyStopFault), false);
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseErrors(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::ERROR, status.level);
}

TEST_F(TestLynxSystem, DiagnoseStatusCountsPDOTimeouts)
{
  using husarion_ugv_hardware_interfaces_test::GetDiagnosticValue;

  husarion_ugv_hardware_interfaces::DriverData data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);
  ON_CALL(*lynx_system_->mock_robot_driver, GetData(::testing::_))
    .WillByDefault(::testing::ReturnRef(data));

  diagnostic_updater::DiagnosticStatusWrapper status;
  lynx_system_->DiagnoseStatus(status);
  EXPECT_EQ("0", GetDiagnosticValue(status, "PDO timeouts in the last hour"));

  // Two separate timeouts, the second lasting three cycles.
  lynx_system_->TickCANopenResyncWatchdog(true);
  lynx_system_->TickCANopenResyncWatchdog(false);
  lynx_system_->TickCANopenResyncWatchdog(true);
  lynx_system_->TickCANopenResyncWatchdog(true);
  lynx_system_->TickCANopenResyncWatchdog(true);

  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseStatus(status);
  EXPECT_EQ("2", GetDiagnosticValue(status, "PDO timeouts in the last hour"));

  // The hour keeps them across reports.
  status.clearSummary();
  status.values.clear();
  lynx_system_->DiagnoseStatus(status);
  EXPECT_EQ("2", GetDiagnosticValue(status, "PDO timeouts in the last hour"));
}

TEST_F(TestLynxSystem, DiagnoseStatusStaleWhenDataUnavailable)
{
  ON_CALL(*lynx_system_->mock_robot_driver, GetData(::testing::_))
    .WillByDefault(::testing::Throw(std::runtime_error("no data")));

  diagnostic_updater::DiagnosticStatusWrapper status;
  lynx_system_->DiagnoseStatus(status);
  EXPECT_EQ(diagnostic_updater::DiagnosticStatusWrapper::STALE, status.level);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  return result;
}
