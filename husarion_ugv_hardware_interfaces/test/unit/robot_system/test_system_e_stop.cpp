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

#include <memory>
#include <mutex>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "hardware_interface/hardware_info.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_error_filter.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_e_stop.hpp"

#include "utils/system_test_utils.hpp"

using husarion_ugv_hardware_interfaces::GPIOPin;
using husarion_ugv_hardware_interfaces_test::MockGPIODriver;
using testing::Return;

class GPIOControllerWithLine : public husarion_ugv_hardware_interfaces_test::MockGPIOController
{
public:
  MockGPIODriver & Driver() { return static_cast<MockGPIODriver &>(*gpio_driver_); }
};

class TestEStop : public testing::Test
{
public:
  TestEStop()
  {
    gpio_controller_ = std::make_shared<testing::NiceMock<GPIOControllerWithLine>>();
    robot_driver_ =
      std::make_shared<testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockRobotDriver>>();
    auto error_filter = std::make_shared<husarion_ugv_hardware_interfaces::RoboteqErrorFilter>(
      2, 2, 2, 2);
    e_stop_ = std::make_unique<husarion_ugv_hardware_interfaces::EStop>(
      gpio_controller_, error_filter, robot_driver_, std::make_shared<std::mutex>(),
      []() { return true; });
  }

protected:
  // The reset pin is active while the hardware line is released.
  void SetLineAsserted(const bool asserted)
  {
    ON_CALL(gpio_controller_->Driver(), IsPinActive(GPIOPin::E_STOP_RESET))
      .WillByDefault(Return(!asserted));
  }

  std::string ReadAndCaptureLog()
  {
    testing::internal::CaptureStderr();
    e_stop_->ReadEStopState();
    return testing::internal::GetCapturedStderr();
  }

  void ReleaseLine()
  {
    SetLineAsserted(false);
    e_stop_->ResetEStop();
    ReadAndCaptureLog();
  }

  std::shared_ptr<testing::NiceMock<GPIOControllerWithLine>> gpio_controller_;
  std::shared_ptr<testing::NiceMock<husarion_ugv_hardware_interfaces_test::MockRobotDriver>>
    robot_driver_;
  std::unique_ptr<husarion_ugv_hardware_interfaces::EStop> e_stop_;
};

TEST_F(TestEStop, LineAssertedWithoutSoftwareTriggerBlamesOutsideDevice)
{
  ReleaseLine();

  SetLineAsserted(true);
  const auto log = ReadAndCaptureLog();
  EXPECT_THAT(log, testing::HasSubstr("No software trigger was requested"));
}

TEST_F(TestEStop, LineAssertedAfterSoftwareTriggerNamesTheTrigger)
{
  ReleaseLine();

  testing::internal::CaptureStderr();
  e_stop_->TriggerEStop();
  testing::internal::GetCapturedStderr();

  // The safety board drops the line a few cycles after the watchdog stops.
  EXPECT_EQ("", ReadAndCaptureLog());
  SetLineAsserted(true);
  const auto log = ReadAndCaptureLog();
  EXPECT_THAT(log, testing::HasSubstr("after the software trigger"));
  EXPECT_THAT(log, testing::Not(testing::HasSubstr("No software trigger")));
}

TEST_F(TestEStop, SoftwareTriggerDoesNotCoverLaterOutsideStops)
{
  ReleaseLine();

  testing::internal::CaptureStderr();
  e_stop_->TriggerEStop();
  testing::internal::GetCapturedStderr();
  SetLineAsserted(true);
  ReadAndCaptureLog();

  ReleaseLine();
  SetLineAsserted(true);
  EXPECT_THAT(ReadAndCaptureLog(), testing::HasSubstr("No software trigger was requested"));
}

TEST_F(TestEStop, ResetClearsAPendingSoftwareTrigger)
{
  // Triggered while the line was already asserted, so no edge consumed it.
  SetLineAsserted(true);
  ReadAndCaptureLog();
  testing::internal::CaptureStderr();
  e_stop_->TriggerEStop();
  testing::internal::GetCapturedStderr();

  ReleaseLine();
  SetLineAsserted(true);
  EXPECT_THAT(ReadAndCaptureLog(), testing::HasSubstr("No software trigger was requested"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
