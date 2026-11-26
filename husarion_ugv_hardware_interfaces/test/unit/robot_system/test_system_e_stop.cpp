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
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "hardware_interface/hardware_info.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_error_filter.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_e_stop.hpp"

#include "utils/system_test_utils.hpp"

using husarion_ugv_hardware_interfaces::GPIOPin;
using husarion_ugv_hardware_interfaces_test::MockGPIOController;
using husarion_ugv_hardware_interfaces_test::MockRobotDriver;
using testing::Invoke;

class EStopWrapper : public husarion_ugv_hardware_interfaces::EStop
{
public:
  using EStop::EStop;

  std::mutex & GetManipulationMutex() { return e_stop_manipulation_mtx_; }
  std::uint64_t GetTriggersRequested() const { return triggers_requested_.load(); }
};

class TestEStop : public testing::Test
{
public:
  TestEStop()
  {
    gpio_controller_ = std::make_shared<MockGPIOController::NiceMock>();
    robot_driver_ = std::make_shared<MockRobotDriver::NiceMock>();
    auto error_filter = std::make_shared<husarion_ugv_hardware_interfaces::RoboteqErrorFilter>(
      2, 2, 2, 2);
    e_stop_ = std::make_unique<EStopWrapper>(
      gpio_controller_, error_filter, robot_driver_, robot_driver_write_mtx_,
      [this]() { return zero_velocity_.load(); }, [this]() { return roboteq_e_stop_set_.load(); });
    // The E-stop line is not latched by the hardware.
    ON_CALL(gpio_controller_->GetMockGPIODriver(), IsPinActive(testing::_))
      .WillByDefault(testing::Return(true));
  }

protected:
  // The reset pin is active while the hardware line is released.
  void SetLineAsserted(const bool asserted)
  {
    ON_CALL(gpio_controller_->GetMockGPIODriver(), IsPinActive(GPIOPin::E_STOP_RESET))
      .WillByDefault(testing::Return(!asserted));
  }

  std::string ReadAndCaptureLog()
  {
    testing::internal::CaptureStderr();
    e_stop_->ReadEStopState();
    return testing::internal::GetCapturedStderr();
  }

  void TriggerQuietly()
  {
    testing::internal::CaptureStderr();
    e_stop_->TriggerEStop();
    testing::internal::GetCapturedStderr();
  }

  void ReleaseLine()
  {
    SetLineAsserted(false);
    e_stop_->ResetEStop();
    ReadAndCaptureLog();
  }

  // Returns false when the trigger was not requested within 5 s.
  bool WaitForTriggerCount(const std::uint64_t count)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (e_stop_->GetTriggersRequested() < count) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::yield();
    }
    return true;
  }

  // Brings the E-stop to the released state, which a freshly constructed EStop is not in.
  void ReleaseEStop()
  {
    ASSERT_NO_THROW(e_stop_->ResetEStop());
    ASSERT_FALSE(e_stop_->ReadEStopState());
    testing::Mock::VerifyAndClearExpectations(robot_driver_.get());
    testing::Mock::VerifyAndClearExpectations(gpio_controller_.get());
  }

  std::shared_ptr<MockGPIOController::NiceMock> gpio_controller_;
  std::shared_ptr<MockRobotDriver::NiceMock> robot_driver_;
  std::shared_ptr<std::mutex> robot_driver_write_mtx_ = std::make_shared<std::mutex>();
  std::unique_ptr<EStopWrapper> e_stop_;
  std::atomic_bool zero_velocity_{true};
  std::atomic_bool roboteq_e_stop_set_{false};
};

TEST_F(TestEStop, ResetWaitsForTheStateRead)
{
  // ReadEStopState() takes the mutex on every read cycle. A reset that arrives meanwhile has to
  // wait for it and reach the GPIO reset, not return as if it had.
  std::promise<void> reset_started;
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).WillOnce(Invoke([&]() {
    reset_started.set_value();
  }));
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(0);
  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(0);

  std::unique_lock<std::mutex> reader_lock(e_stop_->GetManipulationMutex());
  auto reset = std::async(std::launch::async, [&]() { e_stop_->ResetEStop(); });
  reset_started.get_future().wait();
  EXPECT_EQ(reset.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  testing::Mock::VerifyAndClearExpectations(gpio_controller_.get());

  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(1);
  reader_lock.unlock();
  EXPECT_NO_THROW(reset.get());
}

TEST_F(TestEStop, ResetRefusedWhenTriggeredWhileWaiting)
{
  std::promise<void> reset_started;
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).WillOnce(Invoke([&]() {
    reset_started.set_value();
  }));
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);
  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(0);
  EXPECT_CALL(*gpio_controller_, EStopTrigger()).Times(1);

  std::unique_lock<std::mutex> reader_lock(e_stop_->GetManipulationMutex());
  auto reset = std::async(std::launch::async, [&]() { e_stop_->ResetEStop(); });
  reset_started.get_future().wait();

  // The trigger counts itself before it waits for the mutex, so whichever of the two gets the
  // mutex first, the reset must be refused.
  auto trigger = std::async(std::launch::async, [&]() { e_stop_->TriggerEStop(); });
  ASSERT_TRUE(WaitForTriggerCount(1));
  reader_lock.unlock();

  EXPECT_THROW(reset.get(), std::runtime_error);
  EXPECT_NO_THROW(trigger.get());
}

TEST_F(TestEStop, ResetRefusedWhileTriggerIsPending)
{
  // The trigger is requested before the reset starts, but it has not latched the line yet. If the
  // reset went ahead, the trigger would latch the line right after a reset that reported success.
  std::promise<void> release_trigger;
  auto release_trigger_future = release_trigger.get_future().share();
  EXPECT_CALL(*gpio_controller_, InterruptEStopReset()).WillOnce(Invoke([&]() {
    release_trigger_future.wait();
  }));
  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(0);
  EXPECT_CALL(*gpio_controller_, EStopTrigger()).Times(1);
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);

  auto trigger = std::async(std::launch::async, [&]() { e_stop_->TriggerEStop(); });
  ASSERT_TRUE(WaitForTriggerCount(1));

  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);

  release_trigger.set_value();
  EXPECT_NO_THROW(trigger.get());
}

TEST_F(TestEStop, ResetRefusedWhenTriggeredDuringGPIOReset)
{
  // A trigger whose interrupt reached the GPIO reset before it started waiting is not seen by the
  // GPIO reset itself. The count catches it.
  std::future<void> trigger;
  EXPECT_CALL(*gpio_controller_, EStopReset()).WillOnce(Invoke([&]() {
    trigger = std::async(std::launch::async, [&]() { e_stop_->TriggerEStop(); });
    EXPECT_TRUE(WaitForTriggerCount(1));
  }));
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);

  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
  ASSERT_TRUE(trigger.valid());
  EXPECT_NO_THROW(trigger.get());
}

TEST_F(TestEStop, ResetAfterEarlierTrigger)
{
  // Only a trigger after the reset started refuses it.
  e_stop_->TriggerEStop();

  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(1);
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(0);
  EXPECT_NO_THROW(e_stop_->ResetEStop());
}

TEST_F(TestEStop, ResetRefusedWithNonZeroVelocity)
{
  zero_velocity_ = false;

  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(0);
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(1);
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);
  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
}

TEST_F(TestEStop, RefusedResetOfReleasedEStopDoesNotLatch)
{
  // A reset while driving is refused, and must not latch the motor controllers while
  // /hardware/e_stop keeps reporting false. It does not write to them at all, the control loop
  // needs the write mutex.
  ReleaseEStop();
  zero_velocity_ = false;

  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(0);
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(0);
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(0);
  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
  EXPECT_FALSE(e_stop_->ReadEStopState());
}

TEST_F(TestEStop, ResetLatchesAgainWhenReleaseFails)
{
  // On a Panther the front controller can be released when the rear one fails.
  EXPECT_CALL(*robot_driver_, TurnOffEStop())
    .WillOnce(testing::Throw(std::runtime_error("SDO write failed")));
  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(0);
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);
  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
  EXPECT_TRUE(e_stop_->ReadEStopState());
}

TEST_F(TestEStop, HardwareLatchWhileResetWaitsIsLeftToGPIOReset)
{
  // A latch of the line by the safety board does not go through the trigger counters. The reset
  // still goes to the GPIO reset, which fails when the line stays latched.
  ReleaseEStop();

  std::promise<void> read_started;
  std::promise<void> release_read;
  auto release_read_future = release_read.get_future().share();
  auto & gpio_driver = gpio_controller_->GetMockGPIODriver();
  EXPECT_CALL(gpio_driver, IsPinActive(husarion_ugv_hardware_interfaces::GPIOPin::E_STOP_RESET))
    .WillOnce(Invoke([&](auto) {
      read_started.set_value();
      release_read_future.wait();
      return false;
    }))
    .WillRepeatedly(testing::Return(true));
  EXPECT_CALL(*gpio_controller_, EStopTrigger()).Times(1);
  EXPECT_CALL(*gpio_controller_, EStopReset())
    .WillOnce(testing::Throw(std::runtime_error("E-stop is still active")));
  // The reset started on a released E-stop, so the motor controllers were never released by it
  // and there is nothing to latch again. The hardware line is latched either way.
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(0);
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(0);

  auto read = std::async(std::launch::async, [&]() { return e_stop_->ReadEStopState(); });
  read_started.get_future().wait();
  auto reset = std::async(std::launch::async, [&]() { e_stop_->ResetEStop(); });
  EXPECT_EQ(reset.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  release_read.set_value();

  EXPECT_TRUE(read.get());
  EXPECT_THROW(reset.get(), std::runtime_error);
}

TEST_F(TestEStop, RefusedResetOfReleasedEStopByATriggerDoesNotLatch)
{
  // Whether to latch again is decided when the reset starts, not after the trigger that refuses
  // it has set the flags.
  ReleaseEStop();

  std::future<void> trigger;
  EXPECT_CALL(*gpio_controller_, EStopReset()).WillOnce(Invoke([&]() {
    trigger = std::async(std::launch::async, [&]() { e_stop_->TriggerEStop(); });
    EXPECT_TRUE(WaitForTriggerCount(1));
  }));
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(0);

  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
  ASSERT_TRUE(trigger.valid());
  EXPECT_NO_THROW(trigger.get());
  EXPECT_TRUE(e_stop_->ReadEStopState());
}

TEST_F(TestEStop, ResetWritesUnderTheDriverWriteMutex)
{
  // MotorTorqueEnable writes the same objects under this mutex.
  std::promise<void> release_started;
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).WillOnce(Invoke([&]() {
    release_started.set_value();
  }));

  std::unique_lock<std::mutex> torque_lock(*robot_driver_write_mtx_);
  auto reset = std::async(std::launch::async, [&]() { e_stop_->ResetEStop(); });
  auto release_started_future = release_started.get_future();
  EXPECT_EQ(
    release_started_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  torque_lock.unlock();

  EXPECT_EQ(release_started_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_NO_THROW(reset.get());
}

TEST_F(TestEStop, ResetAfterCloseIsRefused)
{
  e_stop_->Close();

  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(0);
  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(0);
  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
  EXPECT_TRUE(e_stop_->ReadEStopState());
}

TEST_F(TestEStop, CloseWaitsForAResetInFlight)
{
  // A reset that already holds the mutex finishes first, and Close() then refuses it, so no reset
  // can turn the watchdog on after Close() returned.
  std::promise<void> gpio_reset_started;
  std::promise<void> finish_gpio_reset;
  auto finish_gpio_reset_future = finish_gpio_reset.get_future().share();
  EXPECT_CALL(*gpio_controller_, EStopReset()).WillOnce(Invoke([&]() {
    gpio_reset_started.set_value();
    finish_gpio_reset_future.wait();
  }));
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);

  auto reset = std::async(std::launch::async, [&]() { e_stop_->ResetEStop(); });
  gpio_reset_started.get_future().wait();
  auto close = std::async(std::launch::async, [&]() { e_stop_->Close(); });
  EXPECT_EQ(close.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  finish_gpio_reset.set_value();

  EXPECT_NO_THROW(close.get());
  EXPECT_THROW(reset.get(), std::runtime_error);
  EXPECT_TRUE(e_stop_->ReadEStopState());
}

TEST_F(TestEStop, LatchedRightAfterARefusalAtTheSecondCheck)
{
  // The line the GPIO reset released stays released until the waiting trigger runs. Until then the
  // software flag, which a refused reset never clears, keeps the state latched.
  std::promise<void> trigger_applying;
  std::promise<void> finish_trigger;
  auto finish_trigger_future = finish_trigger.get_future().share();
  std::future<void> trigger;
  EXPECT_CALL(*gpio_controller_, EStopReset()).WillOnce(Invoke([&]() {
    trigger = std::async(std::launch::async, [&]() { e_stop_->TriggerEStop(); });
    EXPECT_TRUE(WaitForTriggerCount(1));
  }));
  EXPECT_CALL(*gpio_controller_, EStopTrigger()).WillOnce(Invoke([&]() {
    trigger_applying.set_value();
    finish_trigger_future.wait();
  }));

  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
  trigger_applying.get_future().wait();
  EXPECT_TRUE(e_stop_->ReadEStopState());
  finish_trigger.set_value();
  ASSERT_TRUE(trigger.valid());
  EXPECT_NO_THROW(trigger.get());
}

TEST_F(TestEStop, TriggerWithAFailedGPIOWriteStaysLatched)
{
  ReleaseEStop();

  EXPECT_CALL(*gpio_controller_, EStopTrigger())
    .WillOnce(testing::Throw(std::runtime_error("GPIO write failed")));
  EXPECT_THROW(e_stop_->TriggerEStop(), std::runtime_error);
  EXPECT_TRUE(e_stop_->ReadEStopState());

  // The trigger is counted as applied, so a later reset is not refused for it.
  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(1);
  EXPECT_NO_THROW(e_stop_->ResetEStop());
  EXPECT_FALSE(e_stop_->ReadEStopState());
}

TEST_F(TestEStop, ResetOfReleasedEStopReleasesALatchedMotorController)
{
  // A torque disable that raced a successful reset leaves the motor controllers latched behind a
  // released e-stop. Their own fault flag makes the next reset release them.
  ReleaseEStop();
  roboteq_e_stop_set_ = true;

  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(1);
  EXPECT_NO_THROW(e_stop_->ResetEStop());
}

TEST_F(TestEStop, RefusedResetOfReleasedEStopLatchesTheMotorControllerAgain)
{
  // The release went out to a latched motor controller, so a refused reset latches it again.
  ReleaseEStop();
  roboteq_e_stop_set_ = true;
  zero_velocity_ = false;

  testing::InSequence seq;
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).Times(1);
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);
  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
}

TEST_F(TestEStop, RefusedResetCallsTheHookBeforeTheLatch)
{
  testing::MockFunction<void()> before_relatch;
  auto error_filter = std::make_shared<husarion_ugv_hardware_interfaces::RoboteqErrorFilter>(
    2, 2, 2, 2);
  EStopWrapper e_stop(
    gpio_controller_, error_filter, robot_driver_, robot_driver_write_mtx_, []() { return false; },
    []() { return false; }, before_relatch.AsStdFunction());

  testing::InSequence sequence;
  EXPECT_CALL(before_relatch, Call()).Times(1);
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);
  EXPECT_THROW(e_stop.ResetEStop(), std::runtime_error);
}

TEST_F(TestEStop, ResetRefusedWhenGPIOResetFails)
{
  EXPECT_CALL(*gpio_controller_, EStopReset())
    .WillOnce(testing::Throw(std::runtime_error("E-stop reset failed")));
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);
  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
}

TEST_F(TestEStop, ResetLatchesAgainOnAnyException)
{
  // GPIODriver::SetPinValue throws std::invalid_argument, which is not a runtime_error.
  EXPECT_CALL(*gpio_controller_, EStopReset())
    .WillOnce(testing::Throw(std::invalid_argument("Invalid pin")));
  EXPECT_CALL(*robot_driver_, TurnOnEStop()).Times(1);
  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);
}

TEST_F(TestEStop, ResetReportsFailedLatch)
{
  zero_velocity_ = false;

  EXPECT_CALL(*robot_driver_, TurnOnEStop())
    .WillOnce(testing::Throw(std::runtime_error("SDO write failed")));
  try {
    e_stop_->ResetEStop();
    FAIL() << "The reset was not refused.";
  } catch (const std::runtime_error & e) {
    EXPECT_THAT(e.what(), testing::HasSubstr("last velocity commands are not zero"));
    EXPECT_THAT(e.what(), testing::HasSubstr("Latching the motor controllers again failed"));
    EXPECT_THAT(e.what(), testing::HasSubstr("SDO write failed"));
  }
}

TEST_F(TestEStop, LineAssertedWithoutSoftwareTriggerBlamesOutsideDevice)
{
  ReleaseLine();

  SetLineAsserted(true);
  EXPECT_THAT(ReadAndCaptureLog(), testing::HasSubstr("No software trigger was requested"));
}

TEST_F(TestEStop, LineAssertedAfterSoftwareTriggerNamesTheTrigger)
{
  ReleaseLine();
  TriggerQuietly();

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
  TriggerQuietly();
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
  TriggerQuietly();

  ReleaseLine();
  SetLineAsserted(true);
  EXPECT_THAT(ReadAndCaptureLog(), testing::HasSubstr("No software trigger was requested"));
}

TEST_F(TestEStop, DiagnosticsStartLatchedAtStartup)
{
  SetLineAsserted(true);
  ReadAndCaptureLog();

  const auto snapshot = e_stop_->GetDiagnostics()->Get();
  EXPECT_EQ(husarion_ugv_hardware_interfaces::EStopLatchSource::STARTUP, snapshot.source);
  EXPECT_EQ(0, snapshot.last_change_ns);
  EXPECT_TRUE(snapshot.last_refused_reset.empty());
}

TEST_F(TestEStop, DiagnosticsFollowResetAndSoftwareTrigger)
{
  ReleaseLine();
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::RELEASED,
    e_stop_->GetDiagnostics()->Get().source);

  testing::internal::CaptureStderr();
  e_stop_->TriggerEStop();
  testing::internal::GetCapturedStderr();
  SetLineAsserted(true);
  ReadAndCaptureLog();

  const auto snapshot = e_stop_->GetDiagnostics()->Get();
  EXPECT_EQ(husarion_ugv_hardware_interfaces::EStopLatchSource::SOFTWARE, snapshot.source);
  EXPECT_NE(0, snapshot.last_change_ns);
}

TEST_F(TestEStop, DiagnosticsBlameTheHardwareLineWithoutATrigger)
{
  ReleaseLine();

  SetLineAsserted(true);
  ReadAndCaptureLog();
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::HARDWARE,
    e_stop_->GetDiagnostics()->Get().source);

  // A later software trigger does not hide the outside stop.
  testing::internal::CaptureStderr();
  e_stop_->TriggerEStop();
  testing::internal::GetCapturedStderr();
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::HARDWARE,
    e_stop_->GetDiagnostics()->Get().source);
}

TEST_F(TestEStop, DiagnosticsKeepTheRefusedResetReason)
{
  SetLineAsserted(true);
  ReadAndCaptureLog();
  EXPECT_CALL(*gpio_controller_, EStopReset())
    .WillOnce(testing::Throw(std::runtime_error("line stuck")));

  EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error);

  const auto snapshot = e_stop_->GetDiagnostics()->Get();
  EXPECT_EQ(husarion_ugv_hardware_interfaces::EStopLatchSource::STARTUP, snapshot.source);
  EXPECT_THAT(snapshot.last_refused_reset, testing::HasSubstr("line stuck"));
  EXPECT_NE(0, snapshot.last_refused_reset_ns);
}

TEST_F(TestEStop, RepeatedSoftwareTriggerLogsOnce)
{
  // A communication error triggers the E-stop on every read cycle, which must not write a line
  // per cycle from the control loop.
  ReleaseLine();

  testing::internal::CaptureStderr();
  for (int i = 0; i < 100; ++i) {
    e_stop_->TriggerEStop();
  }
  const auto log = testing::internal::GetCapturedStderr();

  const std::string line = "E-Stop triggered in software";
  std::size_t count = 0;
  for (auto pos = log.find(line); pos != std::string::npos; pos = log.find(line, pos + 1)) {
    ++count;
  }
  EXPECT_EQ(1u, count);
}

TEST_F(TestEStop, LatchRightAfterResetIsNotMissed)
{
  // The button is pressed after the reset returned and before a read saw the released line.
  SetLineAsserted(true);
  ReadAndCaptureLog();
  e_stop_->ResetEStop();

  EXPECT_THAT(ReadAndCaptureLog(), testing::HasSubstr("No software trigger was requested"));
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::HARDWARE,
    e_stop_->GetDiagnostics()->Get().source);
}

TEST_F(TestEStop, ResetLogsTheRelease)
{
  SetLineAsserted(true);
  ReadAndCaptureLog();
  SetLineAsserted(false);

  testing::internal::CaptureStderr();
  e_stop_->ResetEStop();
  EXPECT_THAT(testing::internal::GetCapturedStderr(), testing::HasSubstr("line released"));
  EXPECT_EQ("", ReadAndCaptureLog());
}

TEST_F(TestEStop, StartupWithTheLineReleasedStaysLatched)
{
  // The software E-stop is set until the first reset, whatever the line says.
  SetLineAsserted(false);
  testing::internal::CaptureStderr();
  EXPECT_TRUE(e_stop_->ReadEStopState());
  EXPECT_THAT(testing::internal::GetCapturedStderr(), testing::HasSubstr("line released"));
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::STARTUP,
    e_stop_->GetDiagnostics()->Get().source);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
