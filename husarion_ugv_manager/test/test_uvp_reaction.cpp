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

#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"

#include "husarion_ugv_manager/uvp_reaction.hpp"

using husarion_ugv_manager::UvpReaction;
using husarion_ugv_manager::UvpReactionInput;

class TestUvpReaction : public testing::Test
{
protected:
  static constexpr std::int64_t kMs = 1000000;
  // T of 3.75 s, exact in binary so the deadline lands on a whole millisecond.
  static constexpr float kStopTimeoutMin = 0.0625f;

  static UvpReactionInput Uvp(const std::uint32_t trip_id = 1, const std::int64_t stamp = 100)
  {
    UvpReactionInput input;
    input.received = true;
    input.state = UvpReactionInput::kStateUvp;
    input.mode = UvpReactionInput::kModeStop;
    input.reactions_enabled = true;
    input.stop_timeout_min = kStopTimeoutMin;
    input.trip_id = trip_id;
    input.trip_stamp_ns = stamp;
    input.seconds_since_trip = 0.0f;
    return input;
  }

  husarion_ugv_manager::UvpReactionActions Step(
    const UvpReactionInput & input, const std::int64_t ms)
  {
    now_ns_ += ms * kMs;
    return reaction_.Update(input, now_ns_);
  }

  UvpReaction reaction_;
  std::int64_t now_ns_ = 5000 * kMs;
};

TEST_F(TestUvpReaction, NothingWithoutUvp)
{
  UvpReactionInput input;
  EXPECT_FALSE(Step(input, 0).shutdown);
  input = Uvp();
  input.state = 2;
  EXPECT_FALSE(Step(input, 1000).shutdown);
  EXPECT_FALSE(reaction_.Latched());
}

TEST_F(TestUvpReaction, StopShutsDownAfterT)
{
  auto actions = Step(Uvp(), 0);
  EXPECT_FALSE(actions.shutdown);
  EXPECT_FALSE(actions.aux_off);
  EXPECT_TRUE(reaction_.Latched());
  EXPECT_NEAR(3.75, reaction_.SecondsToShutdown(now_ns_).value(), 1e-6);

  EXPECT_FALSE(Step(Uvp(), 3749).shutdown);
  actions = Step(Uvp(), 1);
  EXPECT_TRUE(actions.shutdown);
  EXPECT_FALSE(actions.reason.empty());

  // Once.
  EXPECT_FALSE(Step(Uvp(), 100).shutdown);
  EXPECT_TRUE(reaction_.ShutdownRequested());
}

TEST_F(TestUvpReaction, TIsSeededFromTheTimeSinceTheTrip)
{
  // A safety manager that starts 2 s after the trip has 1.75 s left.
  auto input = Uvp();
  input.seconds_since_trip = 2.0f;
  EXPECT_FALSE(Step(input, 0).shutdown);
  EXPECT_FALSE(Step(input, 1749).shutdown);
  EXPECT_TRUE(Step(input, 1).shutdown);
}

TEST_F(TestUvpReaction, TAlreadyPassedShutsDownAtOnce)
{
  auto input = Uvp();
  input.seconds_since_trip = 10.0f;
  EXPECT_TRUE(Step(input, 0).shutdown);
}

TEST_F(TestUvpReaction, NanTimeSinceTripCountsFromNow)
{
  auto input = Uvp();
  input.seconds_since_trip = std::nanf("");
  EXPECT_FALSE(Step(input, 0).shutdown);
  EXPECT_TRUE(Step(input, 3750).shutdown);
}

TEST_F(TestUvpReaction, ChargingCancelsT)
{
  Step(Uvp(), 0);
  auto charging = Uvp();
  charging.charging = true;
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(Step(charging, 500).shutdown);
  }
  EXPECT_FALSE(reaction_.SecondsToShutdown(now_ns_).has_value());
  EXPECT_TRUE(reaction_.Latched());
}

TEST_F(TestUvpReaction, ChargingEndedWhileLatchedStartsAFullT)
{
  Step(Uvp(), 0);
  auto charging = Uvp();
  charging.charging = true;
  Step(charging, 3000);
  Step(charging, 5000);

  // Unplugged with the protection still latched: a full T from now, not from the trip.
  EXPECT_FALSE(Step(Uvp(), 10).shutdown);
  EXPECT_FALSE(Step(Uvp(), 3749).shutdown);
  EXPECT_TRUE(Step(Uvp(), 1).shutdown);
}

TEST_F(TestUvpReaction, TripWhileChargingWaitsForTheChargingToEnd)
{
  auto charging = Uvp();
  charging.charging = true;
  EXPECT_FALSE(Step(charging, 0).shutdown);
  EXPECT_FALSE(Step(charging, 10000).shutdown);
  EXPECT_FALSE(Step(Uvp(), 0).shutdown);
  EXPECT_TRUE(Step(Uvp(), 3750).shutdown);
}

TEST_F(TestUvpReaction, ShutdownModeShutsDownAtTheTrip)
{
  auto input = Uvp();
  input.mode = UvpReactionInput::kModeShutdown;
  input.aux_off_at_trip = true;
  const auto actions = Step(input, 0);
  EXPECT_TRUE(actions.shutdown);
  // The sequence turns the AUX power off itself, after the hosts.
  EXPECT_FALSE(actions.aux_off);
}

TEST_F(TestUvpReaction, AuxOffAtTripOncePerTrip)
{
  auto input = Uvp();
  input.aux_off_at_trip = true;
  input.charging = true;
  EXPECT_TRUE(Step(input, 0).aux_off);
  EXPECT_FALSE(Step(input, 10).aux_off);

  // A new trip after the protection cleared.
  auto ok = input;
  ok.state = 1;
  Step(ok, 10);
  EXPECT_FALSE(reaction_.Latched());
  // The aux_off_at_trip setting comes with every message.
  EXPECT_FALSE(Step(Uvp(2, 200), 10).aux_off);
  input.trip_id = 3;
  input.trip_stamp_ns = 300;
  Step(ok, 10);
  EXPECT_TRUE(Step(input, 10).aux_off);
}

TEST_F(TestUvpReaction, ReactionsOffDoNothing)
{
  auto input = Uvp();
  input.reactions_enabled = false;
  input.aux_off_at_trip = true;
  for (int i = 0; i < 10; ++i) {
    const auto actions = Step(input, 1000);
    EXPECT_FALSE(actions.shutdown);
    EXPECT_FALSE(actions.aux_off);
  }
  EXPECT_FALSE(reaction_.Latched());
}

TEST_F(TestUvpReaction, ClearedProtectionCancelsT)
{
  Step(Uvp(), 0);
  auto ok = Uvp();
  ok.state = 1;
  Step(ok, 50);
  EXPECT_FALSE(reaction_.SecondsToShutdown(now_ns_).has_value());
  EXPECT_FALSE(Step(ok, 5000).shutdown);
}

TEST_F(TestUvpReaction, RestartedManagerKeepsTheTripsT)
{
  // The trip happened before this manager started; a new instance sees the same trip id with the
  // elapsed time and does not start T again.
  auto input = Uvp();
  input.seconds_since_trip = 3.5f;
  Step(input, 0);
  EXPECT_NEAR(0.25, reaction_.SecondsToShutdown(now_ns_).value(), 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
