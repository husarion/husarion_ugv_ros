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
#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/lynx_robot_driver.hpp>
#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/robot_driver.hpp>

#include "utils/fake_can_socket.hpp"
#include "utils/mock_driver.hpp"
#include "utils/test_constants.hpp"

#include "husarion_ugv_utils/test/test_utils.hpp"

class LynxRobotDriverWrapper : public husarion_ugv_hardware_interfaces::LynxRobotDriver
{
public:
  LynxRobotDriverWrapper(
    const husarion_ugv_hardware_interfaces::CANopenSettings & canopen_settings,
    const husarion_ugv_hardware_interfaces::DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time = std::chrono::milliseconds(1000))
  : LynxRobotDriver(canopen_settings, drivetrain_settings, activate_wait_time)
  {
    mock_left_motor_driver = std::make_shared<
      ::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>();
    mock_right_motor_driver = std::make_shared<
      ::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>();

    mock_driver =
      std::make_shared<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockDriver>>();
    mock_driver->AddMotorDriver(
      husarion_ugv_hardware_interfaces::MotorNames::LEFT, mock_left_motor_driver);
    mock_driver->AddMotorDriver(
      husarion_ugv_hardware_interfaces::MotorNames::RIGHT, mock_right_motor_driver);
  }

  void DefineDrivers() override
  {
    drivers_.emplace(husarion_ugv_hardware_interfaces::DriverNames::DEFAULT, mock_driver);
  }

  std::shared_ptr<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockDriver>>
    mock_driver;
  std::shared_ptr<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>
    mock_left_motor_driver;
  std::shared_ptr<::testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockMotorDriver>>
    mock_right_motor_driver;
};

class TestLynxRobotDriver : public ::testing::Test
{
public:
  TestLynxRobotDriver()
  {
    can_socket_ = std::make_unique<husarion_ugv_hardware_interfaces_test::FakeCANSocket>(
      husarion_ugv_hardware_interfaces_test::kCANopenSettings.can_interface_name);
    can_socket_->Initialize();

    robot_driver_ = std::make_unique<LynxRobotDriverWrapper>(
      husarion_ugv_hardware_interfaces_test::kCANopenSettings,
      husarion_ugv_hardware_interfaces_test::kDrivetrainSettings, std::chrono::milliseconds(10));

    robot_driver_->Initialize();
    robot_driver_->Activate();
  }

  ~TestLynxRobotDriver() { robot_driver_->Deinitialize(); }

protected:
  std::unique_ptr<husarion_ugv_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<LynxRobotDriverWrapper> robot_driver_;
};

TEST_F(TestLynxRobotDriver, SendSpeedCommands)
{
  using husarion_ugv_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float left_v = 0.1;
  const float right_v = 0.2;

  EXPECT_CALL(*robot_driver_->mock_driver, IsCANError()).Times(1);
  EXPECT_CALL(
    *robot_driver_->mock_left_motor_driver,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(left_v * kRadPerSecToRbtqCmd))))
    .Times(1);
  EXPECT_CALL(
    *robot_driver_->mock_right_motor_driver,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(right_v * kRadPerSecToRbtqCmd))))
    .Times(1);

  const std::vector<float> speeds = {left_v, right_v};
  EXPECT_NO_THROW(robot_driver_->SendSpeedCommands(speeds));
}

TEST_F(TestLynxRobotDriver, SendSpeedCommandsSendCmdVelError)
{
  EXPECT_CALL(*robot_driver_->mock_left_motor_driver, SendCmdVel(::testing::_))
    .WillOnce(::testing::Throw(std::runtime_error("")));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { robot_driver_->SendSpeedCommands({0.0, 0.0}); }, "Driver send Roboteq cmd failed:"));
}

TEST_F(TestLynxRobotDriver, SendSpeedCommandsCANError)
{
  EXPECT_CALL(*robot_driver_->mock_driver, IsCANError()).WillOnce(::testing::Return(true));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { robot_driver_->SendSpeedCommands({0.0, 0.0}); },
    "CAN error detected on the Driver when trying to write speed commands."));
}

TEST_F(TestLynxRobotDriver, SendSpeedCommandsInvalidVectorSize)
{
  const std::vector<float> speeds = {0.1, 0.2, 0.3};

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { robot_driver_->SendSpeedCommands(speeds); }, "Invalid speeds vector size"));
}

TEST_F(TestLynxRobotDriver, AttemptErrorFlagReset)
{
  EXPECT_NO_THROW(robot_driver_->AttemptErrorFlagReset());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
