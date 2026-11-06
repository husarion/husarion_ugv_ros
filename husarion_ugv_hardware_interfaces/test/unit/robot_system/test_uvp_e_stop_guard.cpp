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

#include <cstdint>
#include <memory>

#include <gtest/gtest.h>

#include "husarion_ugv_hardware_interfaces/robot_system/uvp_e_stop_guard.hpp"

using husarion_ugv_hardware_interfaces::UvpEStopGuard;
using husarion_ugv_hardware_interfaces::UvpProtectionInput;

class TestUvpEStopGuard : public testing::Test
{
protected:
  static constexpr std::int64_t kMs = 1000000;

  static UvpProtectionInput Uvp(const std::uint32_t trip_id = 1, const std::int64_t stamp = 100)
  {
    UvpProtectionInput input;
    input.received = true;
    input.state = UvpProtectionInput::kStateUvp;
    input.mode = UvpProtectionInput::kModeStop;
    input.reactions_enabled = true;
    // Milliseconds instead of a minute.
    input.drive_window_s = 0.05f;
    input.trip_id = trip_id;
    input.trip_stamp_ns = stamp;
    return input;
  }

  static UvpProtectionInput Ok()
  {
    auto input = Uvp();
    input.state = 1;
    return input;
  }

  // One control loop cycle. The e-stop follows the triggers: a trigger latches it.
  void Tick(const UvpProtectionInput & input, const std::int64_t ms = 10)
  {
    guard_->Update(input, latched_, now_ns_, [this]() {
      ++triggers_;
      if (trigger_fails_) {
        return false;
      }
      latched_ = true;
      return true;
    });
    now_ns_ += ms * kMs;
  }

  std::unique_ptr<UvpEStopGuard> guard_ = std::make_unique<UvpEStopGuard>();
  std::int64_t now_ns_ = 1000 * kMs;
  bool latched_ = false;
  bool trigger_fails_ = false;
  int triggers_ = 0;
};

TEST_F(TestUvpEStopGuard, NothingBeforeAMessage)
{
  Tick(UvpProtectionInput{});
  Tick(UvpProtectionInput{});
  EXPECT_EQ(0, triggers_);
  EXPECT_FALSE(guard_->Active());
}

TEST_F(TestUvpEStopGuard, NothingWhileOk)
{
  for (int i = 0; i < 10; ++i) {
    Tick(Ok());
  }
  EXPECT_EQ(0, triggers_);
}

TEST_F(TestUvpEStopGuard, UvpTriggersOncePerTrip)
{
  for (int i = 0; i < 10; ++i) {
    Tick(Uvp());
  }
  EXPECT_EQ(1, triggers_);
  EXPECT_TRUE(guard_->Active());
  EXPECT_EQ(1u, guard_->GetHandledTripId());
  EXPECT_EQ(1u, guard_->GetTriggerCount());
}

TEST_F(TestUvpEStopGuard, AlreadyLatchedStillTriggers)
{
  // The trigger also records UVP as the cause, so it is made even on a latched e-stop.
  latched_ = true;
  Tick(Uvp());
  EXPECT_EQ(1, triggers_);
}

TEST_F(TestUvpEStopGuard, ReactionsOffNeverTrigger)
{
  auto input = Uvp();
  input.reactions_enabled = false;
  for (int i = 0; i < 10; ++i) {
    Tick(input);
  }
  latched_ = false;
  for (int i = 0; i < 10; ++i) {
    Tick(input);
  }
  EXPECT_EQ(0, triggers_);
  EXPECT_FALSE(guard_->Active());
}

TEST_F(TestUvpEStopGuard, ShutdownModeLeavesTheEStopAlone)
{
  auto input = Uvp();
  input.mode = 1;
  for (int i = 0; i < 10; ++i) {
    Tick(input);
  }
  EXPECT_EQ(0, triggers_);
}

TEST_F(TestUvpEStopGuard, RestartWithALatchedTripTriggers)
{
  Tick(Uvp());
  ASSERT_EQ(1, triggers_);

  // A driver restart starts with a new guard. The battery driver still reports the same trip.
  guard_ = std::make_unique<UvpEStopGuard>();
  latched_ = false;
  Tick(Uvp());
  EXPECT_EQ(2, triggers_);
}

TEST_F(TestUvpEStopGuard, ReleaseOpensTheWindowAndItsEndTriggers)
{
  Tick(Uvp());
  ASSERT_TRUE(latched_);

  // Released by whatever - the guard sees only the e-stop state.
  latched_ = false;
  Tick(Uvp());
  EXPECT_NE(0, guard_->GetWindowEndNs());
  for (int i = 0; i < 4; ++i) {
    Tick(Uvp());
  }
  EXPECT_EQ(1, triggers_);
  EXPECT_FALSE(latched_);

  Tick(Uvp());
  Tick(Uvp());
  EXPECT_EQ(2, triggers_);
  EXPECT_TRUE(latched_);
  EXPECT_EQ(0, guard_->GetWindowEndNs());
}

TEST_F(TestUvpEStopGuard, EveryReleaseOpensANewWindow)
{
  Tick(Uvp());
  for (int window = 0; window < 3; ++window) {
    latched_ = false;
    for (int i = 0; i < 7; ++i) {
      Tick(Uvp());
    }
    EXPECT_TRUE(latched_) << window;
  }
  EXPECT_EQ(4, triggers_);
}

TEST_F(TestUvpEStopGuard, ReleasingAgainInsideTheWindowKeepsItsEnd)
{
  Tick(Uvp());
  latched_ = false;
  Tick(Uvp());
  const auto window_end = guard_->GetWindowEndNs();

  latched_ = true;
  Tick(Uvp());
  latched_ = false;
  Tick(Uvp());
  EXPECT_EQ(window_end, guard_->GetWindowEndNs());

  for (int i = 0; i < 4; ++i) {
    Tick(Uvp());
  }
  EXPECT_EQ(2, triggers_);
}

TEST_F(TestUvpEStopGuard, LatchedAtTheWindowEndIsLeftAsItIs)
{
  Tick(Uvp());
  latched_ = false;
  Tick(Uvp());
  latched_ = true;
  for (int i = 0; i < 10; ++i) {
    Tick(Uvp());
  }
  EXPECT_EQ(1, triggers_);
  EXPECT_EQ(0, guard_->GetWindowEndNs());

  // A later release opens a new window.
  latched_ = false;
  for (int i = 0; i < 7; ++i) {
    Tick(Uvp());
  }
  EXPECT_EQ(2, triggers_);
}

TEST_F(TestUvpEStopGuard, ClearedProtectionClosesTheWindow)
{
  Tick(Uvp());
  latched_ = false;
  Tick(Uvp());
  for (int i = 0; i < 10; ++i) {
    Tick(Ok());
  }
  EXPECT_EQ(1, triggers_);
  EXPECT_FALSE(latched_);
  EXPECT_EQ(0, guard_->GetWindowEndNs());
}

TEST_F(TestUvpEStopGuard, NewTripAfterAClearTriggers)
{
  Tick(Uvp(1, 100));
  latched_ = false;
  Tick(Ok());
  // The battery driver restarted without a latch and counts from 1 again.
  Tick(Uvp(1, 900));
  EXPECT_EQ(2, triggers_);
}

TEST_F(TestUvpEStopGuard, NewTripIdTriggers)
{
  Tick(Uvp(1, 100));
  latched_ = false;
  Tick(Uvp(2, 200));
  EXPECT_EQ(2, triggers_);
}

TEST_F(TestUvpEStopGuard, FailedTriggerIsRetried)
{
  trigger_fails_ = true;
  Tick(Uvp());
  Tick(Uvp());
  EXPECT_EQ(2, triggers_);
  EXPECT_EQ(0u, guard_->GetTriggerCount());
  trigger_fails_ = false;
  Tick(Uvp());
  Tick(Uvp());
  EXPECT_EQ(3, triggers_);
  EXPECT_EQ(1u, guard_->GetTriggerCount());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
