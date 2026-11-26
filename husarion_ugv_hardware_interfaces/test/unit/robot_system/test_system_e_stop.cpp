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
#include <tuple>
#include <utility>
#include <vector>

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
  using EStop::TriggerPending;
};

// Holds the e-stop mutex from another thread, as a state read on a service thread does. The
// control loop thread must never try a mutex it holds itself.
class MutexHolder
{
public:
  explicit MutexHolder(std::mutex & mutex)
  : thread_([this, &mutex]() {
      std::lock_guard<std::mutex> lock(mutex);
      locked_.set_value();
      release_future_.wait();
    })
  {
    locked_.get_future().wait();
  }

  ~MutexHolder() { Release(); }

  void Release()
  {
    if (thread_.joinable()) {
      release_.set_value();
      thread_.join();
    }
  }

private:
  std::promise<void> locked_;
  std::promise<void> release_;
  std::shared_future<void> release_future_ = release_.get_future().share();
  std::thread thread_;
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

TEST_F(TestEStop, UVPTriggerNamesUVPAsTheCause)
{
  ReleaseLine();

  testing::internal::CaptureStderr();
  e_stop_->TriggerEStopFromUVP();
  testing::internal::GetCapturedStderr();
  EXPECT_TRUE(e_stop_->ReadEStopState());
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);
}

TEST_F(TestEStop, UVPTriggerReplacesAnEarlierCause)
{
  ReleaseLine();
  SetLineAsserted(true);
  ReadAndCaptureLog();
  ASSERT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::HARDWARE,
    e_stop_->GetDiagnostics()->Get().source);

  testing::internal::CaptureStderr();
  e_stop_->TriggerEStopFromUVP();
  testing::internal::GetCapturedStderr();
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);
}

TEST_F(TestEStop, UVPTriggerWhileAResetWaitsRefusesTheReset)
{
  // The drive window ending while an operator resets the e-stop: the reset must not undo the
  // trigger, and the trigger must not wait for the reset, it runs in the control loop.
  ReleaseLine();
  // Latched again, so the reset has the motor controllers to release.
  TriggerQuietly();

  std::promise<void> reset_started;
  EXPECT_CALL(*robot_driver_, TurnOffEStop()).WillOnce(Invoke([&]() {
    reset_started.set_value();
  }));
  EXPECT_CALL(*gpio_controller_, EStopReset()).Times(0);

  MutexHolder reader(e_stop_->GetManipulationMutex());
  auto reset = std::async(std::launch::async, [&]() { e_stop_->ResetEStop(); });
  reset_started.get_future().wait();

  // Returns at once while the mutex is held. Pending, it already stops the motion.
  EXPECT_FALSE(e_stop_->TriggerEStopFromUVP());
  EXPECT_TRUE(e_stop_->ReadEStopState());

  reader.Release();
  EXPECT_THROW(
    {
      try {
        reset.get();
      } catch (const std::runtime_error & e) {
        EXPECT_THAT(e.what(), testing::HasSubstr("triggered while the reset was pending"));
        throw;
      }
    },
    std::runtime_error);

  // The refused reset applied the trigger before it released the mutex.
  EXPECT_FALSE(e_stop_->TriggerPending());
  EXPECT_TRUE(e_stop_->ReadEStopState());
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);

  // The guard's retry in the next cycle changes nothing.
  testing::internal::CaptureStderr();
  EXPECT_TRUE(e_stop_->TriggerEStopFromUVP());
  testing::internal::GetCapturedStderr();
  EXPECT_TRUE(e_stop_->ReadEStopState());
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);
}

TEST_F(TestEStop, UVPTriggerDuringAGPIOResetDoesNotWaitForIt)
{
  ReleaseLine();
  SetLineAsserted(true);
  ReadAndCaptureLog();

  std::promise<void> reset_entered;
  std::promise<void> release_reset;
  auto release_reset_future = release_reset.get_future().share();
  EXPECT_CALL(*gpio_controller_, EStopReset()).WillOnce(testing::Invoke([&]() {
    reset_entered.set_value();
    release_reset_future.wait();
  }));

  std::string reset_error;
  std::thread reset([this, &reset_error]() {
    try {
      e_stop_->ResetEStop();
    } catch (const std::runtime_error & e) {
      reset_error = e.what();
    }
  });
  reset_entered.get_future().wait();

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(e_stop_->TriggerEStopFromUVP());
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(200));

  std::atomic_int triggers_after_reset{0};
  ON_CALL(*gpio_controller_, EStopTrigger()).WillByDefault(Invoke([&]() {
    ++triggers_after_reset;
  }));
  release_reset.set_value();
  reset.join();
  EXPECT_THAT(reset_error, testing::HasSubstr("triggered during the reset"));
  // The line the GPIO reset released is latched again before the reset lets go of the mutex.
  EXPECT_EQ(1, triggers_after_reset.load());
  EXPECT_FALSE(e_stop_->TriggerPending());
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);

  testing::internal::CaptureStderr();
  EXPECT_TRUE(e_stop_->TriggerEStopFromUVP());
  testing::internal::GetCapturedStderr();
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);
}

TEST_F(TestEStop, ResetUnderUVPSucceedsAndTheGuardLatchesAgain)
{
  ReleaseLine();
  testing::internal::CaptureStderr();
  ASSERT_TRUE(e_stop_->TriggerEStopFromUVP());
  testing::internal::GetCapturedStderr();

  // No trigger since: the reset releases the e-stop, for the drive window.
  testing::internal::CaptureStderr();
  ASSERT_NO_THROW(e_stop_->ResetEStop());
  testing::internal::GetCapturedStderr();
  EXPECT_FALSE(e_stop_->ReadEStopState());

  // The window ends: the next trigger latches it with UVP as the cause.
  testing::internal::CaptureStderr();
  EXPECT_TRUE(e_stop_->TriggerEStopFromUVP());
  testing::internal::GetCapturedStderr();
  EXPECT_TRUE(e_stop_->ReadEStopState());
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);
}

TEST_F(TestEStop, UVPTriggerAgainstAHolderThatIsNotAReset)
{
  ReleaseLine();
  ASSERT_FALSE(e_stop_->ReadEStopState());

  {
    MutexHolder reader(e_stop_->GetManipulationMutex());
    EXPECT_FALSE(e_stop_->TriggerEStopFromUVP());
    // Not applied, but no motion until it is.
    EXPECT_TRUE(e_stop_->TriggerPending());
    EXPECT_TRUE(e_stop_->ReadEStopState());
  }

  // The protection cleared, so the guard never tries again. The next state read that gets the
  // mutex applies the trigger.
  int gpio_triggers = 0;
  ON_CALL(*gpio_controller_, EStopTrigger()).WillByDefault(Invoke([&]() { ++gpio_triggers; }));
  EXPECT_TRUE(e_stop_->ReadEStopState());
  EXPECT_FALSE(e_stop_->TriggerPending());
  EXPECT_EQ(1, gpio_triggers);
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);

  // Applied once: the next reset is not refused because of it.
  testing::internal::CaptureStderr();
  EXPECT_NO_THROW(e_stop_->ResetEStop());
  testing::internal::GetCapturedStderr();
  EXPECT_FALSE(e_stop_->ReadEStopState());
}

TEST_F(TestEStop, RefusedResetKeepsUVPAsTheCauseOfTheNextLatch)
{
  ReleaseLine();
  SetLineAsserted(true);
  ReadAndCaptureLog();

  std::promise<void> reset_entered;
  std::promise<void> release_reset;
  auto release_reset_future = release_reset.get_future().share();
  EXPECT_CALL(*gpio_controller_, EStopReset()).WillOnce(testing::Invoke([&]() {
    reset_entered.set_value();
    release_reset_future.wait();
  }));
  std::thread reset([this]() { EXPECT_THROW(e_stop_->ResetEStop(), std::runtime_error); });
  reset_entered.get_future().wait();
  EXPECT_FALSE(e_stop_->TriggerEStopFromUVP());
  release_reset.set_value();
  reset.join();
  ASSERT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);

  // The GPIO reset released the line for a moment, then the trigger latches it again.
  SetLineAsserted(false);
  ReadAndCaptureLog();
  SetLineAsserted(true);
  const auto log = ReadAndCaptureLog();
  EXPECT_THAT(log, testing::HasSubstr("after the software trigger"));
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);
}

TEST_F(TestEStop, UVPTriggerWhoseGPIOWriteThrows)
{
  ReleaseLine();
  ON_CALL(*gpio_controller_, EStopTrigger())
    .WillByDefault(testing::Throw(std::runtime_error("GPIO write failed")));

  EXPECT_THROW(e_stop_->TriggerEStopFromUVP(), std::runtime_error);
  // The mutex is free again.
  auto try_lock = std::async(std::launch::async, [this]() {
    std::unique_lock<std::mutex> lock(e_stop_->GetManipulationMutex(), std::try_to_lock);
    return lock.owns_lock();
  });
  EXPECT_TRUE(try_lock.get());
  EXPECT_NE(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);
  // The software flag stops the motion, and the trigger stays pending for the next try.
  EXPECT_TRUE(e_stop_->ReadEStopState());
  EXPECT_TRUE(e_stop_->TriggerPending());

  ON_CALL(*gpio_controller_, EStopTrigger()).WillByDefault(testing::Return());
  testing::internal::CaptureStderr();
  EXPECT_TRUE(e_stop_->TriggerEStopFromUVP());
  testing::internal::GetCapturedStderr();
  EXPECT_FALSE(e_stop_->TriggerPending());
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop_->GetDiagnostics()->Get().source);
}

TEST_F(TestEStop, PendingUVPTriggerStopsMotionWhileAResetHoldsTheMutex)
{
  // A reset of a released e-stop runs its checks with the flags clear, the state a UVP request
  // finds after the reset's last check.
  std::promise<void> in_check;
  std::promise<void> leave_check;
  auto leave_check_future = leave_check.get_future().share();
  std::atomic_bool block_check{false};
  auto e_stop = std::make_unique<EStopWrapper>(
    gpio_controller_,
    std::make_shared<husarion_ugv_hardware_interfaces::RoboteqErrorFilter>(2, 2, 2, 2),
    robot_driver_, std::make_shared<std::mutex>(), [&]() {
      if (block_check.exchange(false)) {
        in_check.set_value();
        leave_check_future.wait();
      }
      return true;
    });
  testing::internal::CaptureStderr();
  ASSERT_NO_THROW(e_stop->ResetEStop());
  testing::internal::GetCapturedStderr();
  ASSERT_FALSE(e_stop->ReadEStopState());

  block_check = true;
  std::string reset_error;
  std::thread reset([&]() {
    try {
      e_stop->ResetEStop();
    } catch (const std::runtime_error & e) {
      reset_error = e.what();
    }
  });
  in_check.get_future().wait();

  EXPECT_FALSE(e_stop->TriggerEStopFromUVP());
  // What write() reads: no velocity command goes out in this cycle.
  EXPECT_TRUE(e_stop->ReadEStopState());

  leave_check.set_value();
  reset.join();
  EXPECT_THAT(reset_error, testing::HasSubstr("triggered during the reset"));
  EXPECT_FALSE(e_stop->TriggerPending());
  EXPECT_TRUE(e_stop->ReadEStopState());
  EXPECT_EQ(
    husarion_ugv_hardware_interfaces::EStopLatchSource::UVP,
    e_stop->GetDiagnostics()->Get().source);
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

TEST_F(TestEStop, ConcurrentResetTorqueUVPAndClose)
{
  // Resets, torque changes and UVP triggers run against each other and a control loop that reads
  // the state, then the e-stop is closed, over many rounds. A reset's start is stamped when it
  // sends the release, after it read the trigger count. Checked throughout:
  // - a read that starts after a UVP trigger returned reads latched, unless a reset that started
  //   after that trigger was called is in flight or succeeded: only such a reset may clear it;
  // - a reset does not succeed when a UVP trigger was called and returned during its GPIO reset,
  //   which is after the reset read the trigger count and before its last check;
  // - no two motor controller writes overlap (the write mutex);
  // - after Close() returns, every read is latched and no GPIO reset runs;
  // - nothing deadlocks.
  std::atomic<std::uint64_t> seq{0};
  // Stamps of the reset in flight; only the reset thread sends a release from a reset.
  static thread_local bool on_reset_thread = false;
  std::atomic<std::uint64_t> reset_started{0};
  std::atomic<std::uint64_t> gpio_reset_started{0};
  std::atomic<std::uint64_t> gpio_reset_ended{0};
  std::atomic_int driver_writes_in_flight{0};
  std::atomic_bool driver_writes_overlapped{false};
  const auto driver_write = [&]() {
    if (++driver_writes_in_flight > 1) {
      driver_writes_overlapped = true;
    }
    std::this_thread::yield();
    --driver_writes_in_flight;
  };
  ON_CALL(*robot_driver_, TurnOnEStop()).WillByDefault(Invoke(driver_write));
  ON_CALL(*robot_driver_, TurnOffEStop()).WillByDefault(Invoke([&]() {
    if (on_reset_thread && reset_started == 0) {
      reset_started = ++seq;
    }
    driver_write();
  }));
  std::atomic_int gpio_resets{0};
  ON_CALL(*gpio_controller_, EStopReset()).WillByDefault(Invoke([&]() {
    gpio_reset_started = ++seq;
    ++gpio_resets;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    gpio_reset_ended = ++seq;
  }));
  roboteq_e_stop_set_ = true;

  for (int round = 0; round < 200; ++round) {
    auto error_filter = std::make_shared<husarion_ugv_hardware_interfaces::RoboteqErrorFilter>(
      2, 2, 2, 2);
    EStopWrapper e_stop(
      gpio_controller_, error_filter, robot_driver_, robot_driver_write_mtx_, []() { return true; },
      [this]() { return roboteq_e_stop_set_.load(); });
    ASSERT_NO_THROW(e_stop.ResetEStop());

    seq = 0;
    reset_started = 0;
    // Each UVP trigger: the sequence number before the call and after its return.
    std::mutex uvp_mtx;
    std::vector<std::pair<std::uint64_t, std::uint64_t>> uvp_calls;
    // The start of the last reset that succeeded.
    std::atomic<std::uint64_t> last_success_started{0};
    std::atomic_bool closed{false};
    std::atomic_bool stop{false};
    std::mutex violation_mtx;
    std::string first_violation;
    std::atomic_int violations{0};
    const auto report = [&](const std::string & what) {
      std::lock_guard<std::mutex> lck(violation_mtx);
      if (violations++ == 0) {
        first_violation = what;
      }
    };

    auto control_loop = std::async(std::launch::async, [&]() {
      while (!stop) {
        const bool closed_before = closed.load();
        std::uint64_t called = 0;
        std::uint64_t returned = 0;
        {
          std::lock_guard<std::mutex> lck(uvp_mtx);
          if (!uvp_calls.empty()) {
            std::tie(called, returned) = uvp_calls.back();
          }
        }
        const auto start = ++seq;
        const bool latched = e_stop.ReadEStopState();
        // In this order: a reset records its success before the next one clears its start.
        const auto in_flight_started = reset_started.load();
        const auto success_started = last_success_started.load();
        if (!latched && closed_before) {
          report("released after Close() returned, read " + std::to_string(start));
        }
        if (
          !latched && returned != 0 && returned < start && in_flight_started < called &&
          success_started < called) {
          report(
            "released after the UVP trigger called at " + std::to_string(called) + " returned at " +
            std::to_string(returned) + ", read " + std::to_string(start) +
            ", reset in flight started at " + std::to_string(in_flight_started) +
            ", last successful reset started at " + std::to_string(success_started));
        }
      }
    });
    auto resets = std::async(std::launch::async, [&]() {
      on_reset_thread = true;
      while (!stop) {
        reset_started = 0;
        gpio_reset_started = 0;
        gpio_reset_ended = 0;
        bool succeeded = false;
        try {
          e_stop.ResetEStop();
          succeeded = true;
        } catch (const std::runtime_error &) {
        }
        if (!succeeded) {
          continue;
        }
        last_success_started = reset_started.load();
        const auto gpio_started = gpio_reset_started.load();
        const auto gpio_ended = gpio_reset_ended.load();
        std::lock_guard<std::mutex> lck(uvp_mtx);
        for (const auto & [called, returned] : uvp_calls) {
          if (called > gpio_started && returned < gpio_ended) {
            report(
              "reset succeeded over the UVP trigger called at " + std::to_string(called) +
              " returned at " + std::to_string(returned) + ", during its GPIO reset " +
              std::to_string(gpio_started) + "-" + std::to_string(gpio_ended));
          }
        }
      }
    });
    auto torque = std::async(std::launch::async, [&]() {
      bool enable = false;
      while (!stop) {
        std::lock_guard<std::mutex> lck(*robot_driver_write_mtx_);
        if (e_stop.ReadEStopState()) {
          enable ? robot_driver_->TurnOffEStop() : robot_driver_->TurnOnEStop();
          enable = !enable;
        }
      }
    });
    auto uvp = std::async(std::launch::async, [&]() {
      for (int i = 0; i < 100; ++i) {
        const auto called = ++seq;
        e_stop.TriggerEStopFromUVP();
        const auto returned = ++seq;
        {
          std::lock_guard<std::mutex> lck(uvp_mtx);
          uvp_calls.emplace_back(called, returned);
        }
        std::this_thread::yield();
      }
    });

    ASSERT_EQ(uvp.wait_for(std::chrono::seconds(30)), std::future_status::ready);
    e_stop.Close();
    const int gpio_resets_at_close = gpio_resets.load();
    closed = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    stop = true;

    for (auto * future : {&control_loop, &resets, &torque}) {
      ASSERT_EQ(future->wait_for(std::chrono::seconds(10)), std::future_status::ready);
      future->get();
    }
    ASSERT_EQ(violations.load(), 0) << "round " << round << ": " << first_violation;
    ASSERT_FALSE(driver_writes_overlapped.load()) << "round " << round;
    ASSERT_EQ(gpio_resets.load(), gpio_resets_at_close) << "round " << round;
    ASSERT_TRUE(e_stop.ReadEStopState()) << "round " << round;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
