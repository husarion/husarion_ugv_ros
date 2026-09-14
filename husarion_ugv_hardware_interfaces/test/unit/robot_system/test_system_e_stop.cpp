// Copyright 2025 Husarion sp. z o.o.
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

#include <memory>
#include <mutex>
#include <stdexcept>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_controller.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_e_stop.hpp"

#include "utils/system_test_utils.hpp"

using husarion_ugv_hardware_interfaces::EStop;
using husarion_ugv_hardware_interfaces::EStopResetInterrupted;
using husarion_ugv_hardware_interfaces::GPIOPin;
using husarion_ugv_hardware_interfaces::RoboteqErrorFilter;
using husarion_ugv_hardware_interfaces_test::MockGPIOController;
using husarion_ugv_hardware_interfaces_test::MockRobotDriver;

class TestEStop : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gpio_controller_ = std::make_shared<MockGPIOController::NiceMock>();
    robot_driver_ = std::make_shared<MockRobotDriver::NiceMock>();
    roboteq_error_filter_ = std::make_shared<RoboteqErrorFilter>(2, 2, 2, 2);
    robot_driver_write_mtx_ = std::make_shared<std::mutex>();

    e_stop_ = std::make_shared<EStop>(
      gpio_controller_, roboteq_error_filter_, robot_driver_, robot_driver_write_mtx_,
      [this]() { return velocity_commands_zero_; });
  }

  void ExpectEStopPinReleased()
  {
    ON_CALL(*gpio_controller_->GetMockGPIODriver(), IsPinActive(GPIOPin::E_STOP_RESET))
      .WillByDefault(::testing::Return(true));
  }

  std::shared_ptr<MockGPIOController::NiceMock> gpio_controller_;
  std::shared_ptr<MockRobotDriver::NiceMock> robot_driver_;
  std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter_;
  std::shared_ptr<std::mutex> robot_driver_write_mtx_;
  std::shared_ptr<EStop> e_stop_;

  bool velocity_commands_zero_ = true;
};

TEST_F(TestEStop, ResetEStopReleasesGPIOBeforeDriverEStop)
{
  ::testing::InSequence seq;
  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(1);
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(1);

  ASSERT_NO_THROW(e_stop_->ResetEStop());
}

TEST_F(TestEStop, ResetEStopNonZeroVelocityCommandsTouchesNothing)
{
  velocity_commands_zero_ = false;

  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(0);
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(0);

  ASSERT_THROW(e_stop_->ResetEStop(), std::runtime_error);
}

TEST_F(TestEStop, ResetEStopGPIOFailureSkipsDriverEStop)
{
  EXPECT_CALL(*gpio_controller_, EStopReset())
    .WillOnce(::testing::Throw(std::runtime_error("GPIO reset failed")));
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(0);

  ASSERT_THROW(e_stop_->ResetEStop(), std::runtime_error);
}

TEST_F(TestEStop, ResetEStopGPIOInterruptedSkipsDriverEStop)
{
  EXPECT_CALL(*gpio_controller_, EStopReset())
    .WillOnce(::testing::Throw(EStopResetInterrupted("interrupted")));
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(0);

  ASSERT_THROW(e_stop_->ResetEStop(), std::runtime_error);
}

TEST_F(TestEStop, ResetEStopKeepsEStopStateWhenDriverEStopFails)
{
  ExpectEStopPinReleased();

  EXPECT_CALL(*robot_driver_, TurnOffEStop())
    .WillOnce(::testing::Throw(std::runtime_error("SDO protocol timed out")));

  ASSERT_THROW(e_stop_->ResetEStop(), std::runtime_error);
  EXPECT_TRUE(e_stop_->ReadEStopState());
}

TEST_F(TestEStop, ResetEStopSucceedsOnRetryAfterDriverEStopFailure)
{
  ExpectEStopPinReleased();

  EXPECT_CALL(*robot_driver_, TurnOffEStop())
    .WillOnce(::testing::Throw(std::runtime_error("SDO protocol timed out")))
    .WillOnce(::testing::Return());

  ASSERT_THROW(e_stop_->ResetEStop(), std::runtime_error);
  ASSERT_NO_THROW(e_stop_->ResetEStop());
  EXPECT_FALSE(e_stop_->ReadEStopState());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
