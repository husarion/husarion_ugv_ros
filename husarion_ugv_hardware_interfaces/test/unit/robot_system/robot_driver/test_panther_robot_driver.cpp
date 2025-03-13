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
#include <memory>
#include <stdexcept>
#include <vector>

#include <gmock/gmock.h>

#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp>
#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp>
#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/panther_robot_driver.hpp>
#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/robot_driver.hpp>

#include "utils/fake_can_socket.hpp"
#include "utils/mock_driver.hpp"
#include "utils/test_constants.hpp"

#include "husarion_ugv_utils/test/test_utils.hpp"

class PantherRobotDriverWrapper : public husarion_ugv_hardware_interfaces::PantherRobotDriver
{
public:
  PantherRobotDriverWrapper(
    const husarion_ugv_hardware_interfaces::CANopenSettings & canopen_settings,
    const husarion_ugv_hardware_interfaces::DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time = std::chrono::milliseconds(1000))
  : PantherRobotDriver(canopen_settings, drivetrain_settings, activate_wait_time)
  {
    mock_fl_motor_driver = std::make_shared<
      ::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>();
    mock_fr_motor_driver = std::make_shared<
      ::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>();
    mock_rl_motor_driver = std::make_shared<
      ::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>();
    mock_rr_motor_driver = std::make_shared<
      ::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>();

    mock_front_driver =
      std::make_shared<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockDriver>>();
    mock_front_driver->AddMotorDriver(
      husarion_ugv_hardware_interfaces::MotorNames::LEFT, mock_fl_motor_driver);
    mock_front_driver->AddMotorDriver(
      husarion_ugv_hardware_interfaces::MotorNames::RIGHT, mock_fr_motor_driver);

    mock_rear_driver =
      std::make_shared<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockDriver>>();
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

  std::shared_ptr<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockDriver>>
    mock_front_driver;
  std::shared_ptr<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockDriver>>
    mock_rear_driver;
  std::shared_ptr<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>
    mock_fl_motor_driver;
  std::shared_ptr<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>
    mock_fr_motor_driver;
  std::shared_ptr<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>
    mock_rl_motor_driver;
  std::shared_ptr<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>
    mock_rr_motor_driver;
};

class TestPantherRobotDriver : public ::testing::Test
{
public:
  TestPantherRobotDriver()
  {
    can_socket_ = std::make_unique<husarion_ugv_hardware_interfaces_test::FakeCANSocket>(
      husarion_ugv_hardware_interfaces_test::kCANopenSettings.can_interface_name);
    can_socket_->Initialize();

    robot_driver_ = std::make_unique<PantherRobotDriverWrapper>(
      husarion_ugv_hardware_interfaces_test::kCANopenSettings,
      husarion_ugv_hardware_interfaces_test::kDrivetrainSettings, std::chrono::milliseconds(10));

    robot_driver_->Initialize();
    robot_driver_->Activate();
  }

  ~TestPantherRobotDriver() { robot_driver_->Deinitialize(); }

protected:
  std::unique_ptr<husarion_ugv_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<PantherRobotDriverWrapper> robot_driver_;
};

TEST_F(TestPantherRobotDriver, SendSpeedCommands)
{
  using husarion_ugv_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float fl_v = 0.1;
  const float fr_v = 0.2;
  const float rl_v = 0.3;
  const float rr_v = 0.4;

  EXPECT_CALL(*robot_driver_->mock_front_driver, IsCANError()).Times(1);
  EXPECT_CALL(*robot_driver_->mock_rear_driver, IsCANError()).Times(1);
  EXPECT_CALL(
    *robot_driver_->mock_fl_motor_driver,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(fl_v * kRadPerSecToRbtqCmd))))
    .Times(1);
  EXPECT_CALL(
    *robot_driver_->mock_fr_motor_driver,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(fr_v * kRadPerSecToRbtqCmd))))
    .Times(1);
  EXPECT_CALL(
    *robot_driver_->mock_rl_motor_driver,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(rl_v * kRadPerSecToRbtqCmd))))
    .Times(1);
  EXPECT_CALL(
    *robot_driver_->mock_rr_motor_driver,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(rr_v * kRadPerSecToRbtqCmd))))
    .Times(1);

  const std::vector<float> speeds = {fl_v, fr_v, rl_v, rr_v};
  EXPECT_NO_THROW(robot_driver_->SendSpeedCommands(speeds));
}

TEST_F(TestPantherRobotDriver, SendSpeedCommandsSendCmdVelError)
{
  EXPECT_CALL(*robot_driver_->mock_fl_motor_driver, SendCmdVel(::testing::_))
    .WillOnce(::testing::Throw(std::runtime_error("")));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { robot_driver_->SendSpeedCommands({0.0, 0.0, 0.0, 0.0}); },
    "Front driver send Roboteq cmd failed:"));
}

TEST_F(TestPantherRobotDriver, SendSpeedCommandsCANError)
{
  EXPECT_CALL(*robot_driver_->mock_front_driver, IsCANError()).WillOnce(::testing::Return(true));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { robot_driver_->SendSpeedCommands({0.0, 0.0, 0.0, 0.0}); },
    "CAN error detected on the front driver when trying to write speed commands."));
}

TEST_F(TestPantherRobotDriver, SendSpeedCommandsInvalidVectorSize)
{
  const std::vector<float> speeds = {0.1, 0.2, 0.3};

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { robot_driver_->SendSpeedCommands(speeds); }, "Invalid speeds vector size"));
}

TEST_F(TestPantherRobotDriver, AttemptErrorFlagReset)
{
  EXPECT_NO_THROW(robot_driver_->AttemptErrorFlagReset());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
