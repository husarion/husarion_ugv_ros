// Copyright 2023 Husarion sp. z o.o.
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
#include <cstdint>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <gmock/gmock.h>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/robot_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_robot_driver.hpp"

#include "utils/fake_can_socket.hpp"
#include "utils/mock_driver.hpp"
#include "utils/test_constants.hpp"

#include "husarion_ugv_utils/test/test_utils.hpp"

class RoboteqRobotDriverWrapper : public husarion_ugv_hardware_interfaces::RoboteqRobotDriver
{
public:
  RoboteqRobotDriverWrapper(
    const husarion_ugv_hardware_interfaces::CANopenSettings & canopen_settings,
    const husarion_ugv_hardware_interfaces::DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time = std::chrono::milliseconds(1000))
  : RoboteqRobotDriver(canopen_settings, drivetrain_settings, activate_wait_time)
  {
    // Assume 2 drivers and 4 motor drivers
    mock_fl_motor_driver =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockMotorDriver::NiceMock>();
    mock_fr_motor_driver =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockMotorDriver::NiceMock>();
    mock_rl_motor_driver =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockMotorDriver::NiceMock>();
    mock_rr_motor_driver =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockMotorDriver::NiceMock>();

    mock_front_driver =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockDriver::NiceMock>();
    mock_front_driver->AddMotorDriver(
      husarion_ugv_hardware_interfaces::MotorNames::LEFT, mock_fl_motor_driver);
    mock_front_driver->AddMotorDriver(
      husarion_ugv_hardware_interfaces::MotorNames::RIGHT, mock_fr_motor_driver);

    mock_rear_driver =
      std::make_shared<husarion_ugv_hardware_interfaces_test::MockDriver::NiceMock>();
    mock_rear_driver->AddMotorDriver(
      husarion_ugv_hardware_interfaces::MotorNames::LEFT, mock_rl_motor_driver);
    mock_rear_driver->AddMotorDriver(
      husarion_ugv_hardware_interfaces::MotorNames::RIGHT, mock_rr_motor_driver);
  }

  void DefineDrivers() override
  {
    drivers_.emplace(husarion_ugv_hardware_interfaces::DriverNames::FRONT, mock_front_driver);
    drivers_.emplace(husarion_ugv_hardware_interfaces::DriverNames::REAR, mock_rear_driver);
  }

  void SendSpeedCommands(const std::vector<float> & /*velocities*/) override {}
  void AttemptErrorFlagReset() override {}

  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockDriver::NiceMock> mock_front_driver;
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockDriver::NiceMock> mock_rear_driver;
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockMotorDriver::NiceMock>
    mock_fl_motor_driver;
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockMotorDriver::NiceMock>
    mock_fr_motor_driver;
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockMotorDriver::NiceMock>
    mock_rl_motor_driver;
  std::shared_ptr<husarion_ugv_hardware_interfaces_test::MockMotorDriver::NiceMock>
    mock_rr_motor_driver;
};

class TestRoboteqRobotDriverInitialization : public ::testing::Test
{
public:
  TestRoboteqRobotDriverInitialization()
  {
    can_socket_ = std::make_unique<husarion_ugv_hardware_interfaces_test::FakeCANSocket>(
      husarion_ugv_hardware_interfaces_test::kCANopenSettings.can_interface_name);
    can_socket_->Initialize();

    robot_driver_ = std::make_unique<RoboteqRobotDriverWrapper>(
      husarion_ugv_hardware_interfaces_test::kCANopenSettings,
      husarion_ugv_hardware_interfaces_test::kDrivetrainSettings, std::chrono::milliseconds(10));
  }

  ~TestRoboteqRobotDriverInitialization() {}

protected:
  std::unique_ptr<husarion_ugv_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<RoboteqRobotDriverWrapper> robot_driver_;
};

class TestRoboteqRobotDriver : public TestRoboteqRobotDriverInitialization
{
public:
  TestRoboteqRobotDriver()
  {
    robot_driver_->Initialize();
    robot_driver_->Activate();
  }

  ~TestRoboteqRobotDriver() { robot_driver_->Deinitialize(); }

  timespec GetCurrentTimeWithTimeout(const std::chrono::nanoseconds & timeout_ns)
  {
    timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    current_time.tv_nsec -= timeout_ns.count();

    return current_time;
  }
};

TEST_F(TestRoboteqRobotDriverInitialization, Initialize)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, Boot()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rear_driver, Boot()).Times(1);

  EXPECT_NO_THROW(robot_driver_->Initialize());
  ASSERT_NO_THROW(robot_driver_->Deinitialize());

  EXPECT_CALL(*robot_driver_->mock_front_driver, Boot()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rear_driver, Boot()).Times(1);
  // Check if deinitialization worked correctly - initialize once again
  EXPECT_NO_THROW(robot_driver_->Initialize());
}

TEST_F(TestRoboteqRobotDriverInitialization, Activate)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, ResetScript()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rear_driver, ResetScript()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_fl_motor_driver, SendCmdVel(::testing::Eq(0))).Times(1);
  EXPECT_CALL(*robot_driver_->mock_fr_motor_driver, SendCmdVel(::testing::Eq(0))).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rl_motor_driver, SendCmdVel(::testing::Eq(0))).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rr_motor_driver, SendCmdVel(::testing::Eq(0))).Times(1);

  ASSERT_NO_THROW(robot_driver_->Initialize());
  ASSERT_NO_THROW(robot_driver_->Activate());
}

TEST_F(TestRoboteqRobotDriver, GetData)
{
  EXPECT_NO_THROW(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT));
  EXPECT_NO_THROW(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR));
}

TEST_F(TestRoboteqRobotDriver, UpdateCommunicationState)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, IsCANError()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_front_driver, IsHeartbeatTimeout()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rear_driver, IsCANError()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rear_driver, IsHeartbeatTimeout()).Times(1);

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());
}

TEST_F(TestRoboteqRobotDriver, UpdateCommunicationStateCANErorr)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, IsCANError()).WillOnce(::testing::Return(true));
  EXPECT_CALL(*robot_driver_->mock_rear_driver, IsCANError()).WillOnce(::testing::Return(true));

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());

  EXPECT_TRUE(
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT).IsCANError());
  EXPECT_TRUE(
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR).IsCANError());
}

TEST_F(TestRoboteqRobotDriver, UpdateCommunicationStateHeartbeatTimeout)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, IsHeartbeatTimeout())
    .WillOnce(::testing::Return(true));
  EXPECT_CALL(*robot_driver_->mock_rear_driver, IsHeartbeatTimeout())
    .WillOnce(::testing::Return(true));

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());

  EXPECT_TRUE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT)
                .IsHeartbeatTimeout());
  EXPECT_TRUE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR)
                .IsHeartbeatTimeout());
}

TEST_F(TestRoboteqRobotDriver, UpdateMotorsState)
{
  using husarion_ugv_hardware_interfaces::MotorChannels;
  using husarion_ugv_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  using husarion_ugv_hardware_interfaces_test::kRbtqPosFbToRad;
  using husarion_ugv_hardware_interfaces_test::kRbtqVelFbToRadPerSec;

  const std::int32_t fl_pos = 101;
  const std::int32_t fl_vel = 102;
  const std::int32_t fl_current = 103;
  const std::int32_t fr_pos = 201;
  const std::int32_t fr_vel = 202;
  const std::int32_t fr_current = 203;
  const std::int32_t rl_pos = 301;
  const std::int32_t rl_vel = 302;
  const std::int32_t rl_current = 303;
  const std::int32_t rr_pos = 401;
  const std::int32_t rr_vel = 402;
  const std::int32_t rr_current = 403;

  ON_CALL(*robot_driver_->mock_fl_motor_driver, ReadState())
    .WillByDefault(::testing::Return(husarion_ugv_hardware_interfaces::MotorDriverState(
      {fl_pos, fl_vel, fl_current, {0, 0}, {0, 0}})));
  ON_CALL(*robot_driver_->mock_fr_motor_driver, ReadState())
    .WillByDefault(::testing::Return(husarion_ugv_hardware_interfaces::MotorDriverState(
      {fr_pos, fr_vel, fr_current, {0, 0}, {0, 0}})));
  ON_CALL(*robot_driver_->mock_rl_motor_driver, ReadState())
    .WillByDefault(::testing::Return(husarion_ugv_hardware_interfaces::MotorDriverState(
      {rl_pos, rl_vel, rl_current, {0, 0}, {0, 0}})));
  ON_CALL(*robot_driver_->mock_rr_motor_driver, ReadState())
    .WillByDefault(::testing::Return(husarion_ugv_hardware_interfaces::MotorDriverState(
      {rr_pos, rr_vel, rr_current, {0, 0}, {0, 0}})));

  robot_driver_->UpdateMotorsState();

  const auto & fl_data = robot_driver_
                           ->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT)
                           .GetMotorState(MotorChannels::LEFT);
  const auto & fr_data = robot_driver_
                           ->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT)
                           .GetMotorState(MotorChannels::RIGHT);
  const auto & rl_data = robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR)
                           .GetMotorState(MotorChannels::LEFT);
  const auto & rr_data = robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR)
                           .GetMotorState(MotorChannels::RIGHT);

  EXPECT_FLOAT_EQ(fl_data.GetPosition(), fl_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(fl_data.GetVelocity(), fl_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(fl_data.GetTorque(), fl_current * kRbtqCurrentFbToNewtonMeters);

  EXPECT_FLOAT_EQ(fr_data.GetPosition(), fr_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(fr_data.GetVelocity(), fr_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(fr_data.GetTorque(), fr_current * kRbtqCurrentFbToNewtonMeters);

  EXPECT_FLOAT_EQ(rl_data.GetPosition(), rl_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(rl_data.GetVelocity(), rl_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(rl_data.GetTorque(), rl_current * kRbtqCurrentFbToNewtonMeters);

  EXPECT_FLOAT_EQ(rr_data.GetPosition(), rr_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(rr_data.GetVelocity(), rr_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(rr_data.GetTorque(), rr_current * kRbtqCurrentFbToNewtonMeters);
}

TEST_F(TestRoboteqRobotDriver, UpdateMotorsStateTimestamps)
{
  auto current_time = GetCurrentTimeWithTimeout(
    husarion_ugv_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms);

  auto read_motor_driver_state_method = [&current_time]() {
    husarion_ugv_hardware_interfaces::MotorDriverState state;
    state.pos_timestamp = current_time;
    state.vel_current_timestamp = current_time;
    return state;
  };

  ON_CALL(*robot_driver_->mock_fl_motor_driver, ReadState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));
  ON_CALL(*robot_driver_->mock_fr_motor_driver, ReadState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));
  ON_CALL(*robot_driver_->mock_rl_motor_driver, ReadState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));
  ON_CALL(*robot_driver_->mock_rr_motor_driver, ReadState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));

  robot_driver_->UpdateMotorsState();

  // Update current time to exceed timeout
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  robot_driver_->UpdateMotorsState();

  EXPECT_FALSE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT)
                 .IsMotorStatesDataTimedOut());
  EXPECT_FALSE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR)
                 .IsMotorStatesDataTimedOut());
}

TEST_F(TestRoboteqRobotDriver, UpdateMotorsStateTimeout)
{
  const auto current_time = GetCurrentTimeWithTimeout(
    husarion_ugv_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms);

  husarion_ugv_hardware_interfaces::MotorDriverState state = {0, 0, 0, current_time, current_time};

  ON_CALL(*robot_driver_->mock_fl_motor_driver, ReadState())
    .WillByDefault(::testing::Return(state));
  ON_CALL(*robot_driver_->mock_fr_motor_driver, ReadState())
    .WillByDefault(::testing::Return(state));
  ON_CALL(*robot_driver_->mock_rl_motor_driver, ReadState())
    .WillByDefault(::testing::Return(state));
  ON_CALL(*robot_driver_->mock_rr_motor_driver, ReadState())
    .WillByDefault(::testing::Return(state));

  robot_driver_->UpdateMotorsState();

  EXPECT_TRUE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT)
                .IsMotorStatesDataTimedOut());
  EXPECT_TRUE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR)
                .IsMotorStatesDataTimedOut());
  EXPECT_TRUE(
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT).IsError());
  EXPECT_TRUE(
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR).IsError());
}

TEST_F(TestRoboteqRobotDriver, UpdateDriverState)
{
  using husarion_ugv_hardware_interfaces::MotorChannels;

  const std::int16_t f_temp = 30;
  const std::int16_t r_temp = 32;
  const std::int16_t f_heatsink_temp = 31;
  const std::int16_t r_heatsink_temp = 33;
  const std::uint16_t f_volt = 400;
  const std::uint16_t r_volt = 430;
  const std::int16_t f_battery_current_1 = 10;
  const std::int16_t r_battery_current_1 = 30;
  const std::int16_t f_battery_current_2 = 30;
  const std::int16_t r_battery_current_2 = 40;

  const std::uint8_t fault_flag_overheat = static_cast<std::uint8_t>(0b01);
  const std::uint8_t fault_flag_overvoltage = static_cast<std::uint8_t>(0b10);
  const std::uint8_t script_flag_encoder_disconnected = static_cast<std::uint8_t>(0b10);
  const std::uint8_t script_flag_amp_limiter = static_cast<std::uint8_t>(0b100);
  const std::uint8_t runtime_error_loop_error = static_cast<std::uint8_t>(0b100);
  const std::uint8_t runtime_error_safety_stop_active = static_cast<std::uint8_t>(0b1000);
  const std::uint8_t runtime_error_forward_limit_triggered = static_cast<std::uint8_t>(0b10000);
  const std::uint8_t runtime_error_reverse_limit_triggered = static_cast<std::uint8_t>(0b100000);

  husarion_ugv_hardware_interfaces::DriverState front_driver_state_data = {
    fault_flag_overheat,
    script_flag_encoder_disconnected,
    runtime_error_loop_error,
    runtime_error_safety_stop_active,
    f_battery_current_1,
    f_battery_current_2,
    f_volt,
    f_temp,
    f_heatsink_temp,
    {0, 0},
    {0, 0}};

  husarion_ugv_hardware_interfaces::DriverState rear_driver_state_data = {
    fault_flag_overvoltage,
    script_flag_amp_limiter,
    runtime_error_forward_limit_triggered,
    runtime_error_reverse_limit_triggered,
    r_battery_current_1,
    r_battery_current_2,
    r_volt,
    r_temp,
    r_heatsink_temp,
    {0, 0},
    {0, 0}};

  ON_CALL(*robot_driver_->mock_front_driver, ReadState())
    .WillByDefault(
      ::testing::Return(husarion_ugv_hardware_interfaces::DriverState(front_driver_state_data)));
  ON_CALL(*robot_driver_->mock_rear_driver, ReadState())
    .WillByDefault(
      ::testing::Return(husarion_ugv_hardware_interfaces::DriverState(rear_driver_state_data)));

  robot_driver_->UpdateDriversState();

  const auto & front_data =
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT);
  const auto & rear_data =
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR);
  const auto & front_driver_state =
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT).GetDriverState();
  const auto & rear_driver_state =
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR).GetDriverState();

  EXPECT_EQ(static_cast<std::int16_t>(front_driver_state.GetTemperature()), f_temp);
  EXPECT_EQ(
    static_cast<std::int16_t>(front_driver_state.GetHeatsinkTemperature()), f_heatsink_temp);
  EXPECT_EQ(static_cast<std::uint16_t>(front_driver_state.GetVoltage() * 10.0), f_volt);
  EXPECT_EQ(
    static_cast<std::int16_t>(front_driver_state.GetCurrent() * 10.0),
    f_battery_current_1 + f_battery_current_2);

  EXPECT_EQ(static_cast<std::int16_t>(rear_driver_state.GetTemperature()), r_temp);
  EXPECT_EQ(static_cast<std::int16_t>(rear_driver_state.GetHeatsinkTemperature()), r_heatsink_temp);
  EXPECT_EQ(static_cast<std::uint16_t>(rear_driver_state.GetVoltage() * 10.0), r_volt);
  EXPECT_EQ(
    static_cast<std::int16_t>(rear_driver_state.GetCurrent() * 10.0),
    r_battery_current_1 + r_battery_current_2);

  EXPECT_TRUE(front_data.GetFaultFlag().GetMessage().overheat);
  EXPECT_TRUE(front_data.GetScriptFlag().GetMessage().encoder_disconnected);
  EXPECT_TRUE(front_data.GetRuntimeError(MotorChannels::RIGHT).GetMessage().loop_error);
  EXPECT_TRUE(front_data.GetRuntimeError(MotorChannels::LEFT).GetMessage().safety_stop_active);

  EXPECT_TRUE(rear_data.GetFaultFlag().GetMessage().overvoltage);
  EXPECT_TRUE(rear_data.GetScriptFlag().GetMessage().amp_limiter);
  EXPECT_TRUE(rear_data.GetRuntimeError(MotorChannels::RIGHT).GetMessage().forward_limit_triggered);
  EXPECT_TRUE(rear_data.GetRuntimeError(MotorChannels::LEFT).GetMessage().reverse_limit_triggered);
}

TEST_F(TestRoboteqRobotDriver, UpdateDriverStateTimestamps)
{
  auto current_time = GetCurrentTimeWithTimeout(
    husarion_ugv_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms);

  auto read_driver_state_method = [&current_time]() {
    husarion_ugv_hardware_interfaces::DriverState state;
    state.flags_current_timestamp = current_time;
    state.voltages_temps_timestamp = current_time;
    return state;
  };

  ON_CALL(*robot_driver_->mock_front_driver, ReadState())
    .WillByDefault(::testing::Invoke(read_driver_state_method));
  ON_CALL(*robot_driver_->mock_rear_driver, ReadState())
    .WillByDefault(::testing::Invoke(read_driver_state_method));

  robot_driver_->UpdateDriversState();

  // Update current time to exceed timeout
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  robot_driver_->UpdateDriversState();

  EXPECT_FALSE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT)
                 .IsDriverStateDataTimedOut());
  EXPECT_FALSE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR)
                 .IsDriverStateDataTimedOut());
}

TEST_F(TestRoboteqRobotDriver, UpdateDriverStateTimeout)
{
  const auto current_time = GetCurrentTimeWithTimeout(
    husarion_ugv_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms);

  husarion_ugv_hardware_interfaces::DriverState state = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, current_time, current_time};

  ON_CALL(*robot_driver_->mock_front_driver, ReadState()).WillByDefault(::testing::Return(state));
  ON_CALL(*robot_driver_->mock_rear_driver, ReadState()).WillByDefault(::testing::Return(state));

  robot_driver_->UpdateDriversState();

  EXPECT_TRUE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT)
                .IsDriverStateDataTimedOut());
  EXPECT_TRUE(robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR)
                .IsDriverStateDataTimedOut());
  EXPECT_TRUE(
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::FRONT).IsError());
  EXPECT_TRUE(
    robot_driver_->GetData(husarion_ugv_hardware_interfaces::DriverNames::REAR).IsError());
}

TEST_F(TestRoboteqRobotDriver, TurnOnEStop)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, TurnOnEStop()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rear_driver, TurnOnEStop()).Times(1);

  EXPECT_NO_THROW(robot_driver_->TurnOnEStop());
}

TEST_F(TestRoboteqRobotDriver, TurnOffEStop)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, TurnOffEStop()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rear_driver, TurnOffEStop()).Times(1);

  EXPECT_NO_THROW(robot_driver_->TurnOffEStop());
}

TEST_F(TestRoboteqRobotDriver, TurnOnEStopError)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, TurnOnEStop())
    .WillOnce(::testing::Throw(std::runtime_error("")));
  EXPECT_THROW(robot_driver_->TurnOnEStop(), std::runtime_error);
}

TEST_F(TestRoboteqRobotDriver, TurnOffEStopError)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, TurnOffEStop())
    .WillOnce(::testing::Throw(std::runtime_error("")));
  EXPECT_THROW(robot_driver_->TurnOffEStop(), std::runtime_error);
}

TEST_F(TestRoboteqRobotDriver, CommunicationError)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, IsHeartbeatTimeout())
    .WillOnce(::testing::Return(false));
  EXPECT_CALL(*robot_driver_->mock_rear_driver, IsHeartbeatTimeout())
    .WillOnce(::testing::Return(false));
  EXPECT_CALL(*robot_driver_->mock_front_driver, IsCANError()).WillOnce(::testing::Return(false));
  EXPECT_CALL(*robot_driver_->mock_rear_driver, IsCANError()).WillOnce(::testing::Return(false));

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());
  EXPECT_FALSE(robot_driver_->CommunicationError());

  EXPECT_CALL(*robot_driver_->mock_front_driver, IsHeartbeatTimeout())
    .WillOnce(::testing::Return(true));
  EXPECT_CALL(*robot_driver_->mock_rear_driver, IsHeartbeatTimeout())
    .WillOnce(::testing::Return(true));
  EXPECT_CALL(*robot_driver_->mock_front_driver, IsCANError()).WillOnce(::testing::Return(true));
  EXPECT_CALL(*robot_driver_->mock_rear_driver, IsCANError()).WillOnce(::testing::Return(true));

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());
  EXPECT_TRUE(robot_driver_->CommunicationError());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
