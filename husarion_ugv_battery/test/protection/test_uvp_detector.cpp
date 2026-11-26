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
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "husarion_ugv_battery/protection/uvp_detector.hpp"

using husarion_ugv_battery::UvpDetector;
using husarion_ugv_battery::UvpDetectorConfig;
using husarion_ugv_battery::UvpLatch;
using husarion_ugv_battery::UvpState;

class TestUvpDetector : public testing::Test
{
protected:
  static constexpr std::int64_t kMs = 1000000;

  void Create(const std::size_t battery_count = 1, const UvpDetectorConfig & config = Config())
  {
    detector_ = std::make_unique<UvpDetector>(config, battery_count, Clocks());
  }

  static UvpDetectorConfig Config()
  {
    UvpDetectorConfig config;
    config.uvp_threshold = 31.0f;
    config.charge_asap_offset = 0.5f;
    config.consecutive_samples = 2;
    config.clear_hysteresis = 0.5f;
    // Milliseconds instead of seconds, the clocks below are fake.
    config.charging_debounce_s = 0.005;
    config.blind_timeout_s = 0.002;
    return config;
  }

  UvpDetector::Clocks Clocks()
  {
    UvpDetector::Clocks clocks;
    clocks.steady_ns = [this]() { return steady_ns_; };
    clocks.boottime_ns = [this]() { return boottime_ns_; };
    clocks.ros_ns = [this]() { return ros_ns_; };
    return clocks;
  }

  void Advance(const std::int64_t ms)
  {
    steady_ns_ += ms * kMs;
    boottime_ns_ += ms * kMs;
    ros_ns_ += ms * kMs;
  }

  void Feed(const float voltage, const std::size_t battery = 0, const bool temperature_valid = true)
  {
    detector_->FeedVoltage(battery, voltage, temperature_valid);
    Advance(1);
  }

  void Charge(const bool charging, const std::int64_t ms, const std::size_t battery = 0)
  {
    for (std::int64_t i = 0; i <= ms; ++i) {
      detector_->FeedCharging(battery, charging);
      if (i < ms) {
        Advance(1);
      }
    }
  }

  std::int64_t steady_ns_ = 1000 * kMs;
  std::int64_t boottime_ns_ = 50000 * kMs;
  std::int64_t ros_ns_ = 1700000000000 * kMs;
  std::unique_ptr<UvpDetector> detector_;
};

TEST_F(TestUvpDetector, UnknownUntilFirstValidRead)
{
  Create();
  EXPECT_EQ(UvpState::UNKNOWN, detector_->GetState());
  Feed(35.0f, 0, false);
  EXPECT_EQ(UvpState::UNKNOWN, detector_->GetState());
  Feed(35.0f);
  EXPECT_EQ(UvpState::OK, detector_->GetState());
}

TEST_F(TestUvpDetector, TwoConsecutiveReadsTrip)
{
  Create();
  Feed(30.0f);
  EXPECT_EQ(UvpState::OK, detector_->GetState());
  Feed(30.0f);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
  EXPECT_EQ(1u, detector_->GetTripId());
  EXPECT_FLOAT_EQ(30.0f, detector_->GetTripVoltage(0));
}

TEST_F(TestUvpDetector, OneReadDoesNotTrip)
{
  Create();
  Feed(30.0f);
  Feed(35.0f);
  Feed(30.0f);
  Feed(35.0f);
  EXPECT_EQ(UvpState::OK, detector_->GetState());
  EXPECT_EQ(0u, detector_->GetTripId());
}

TEST_F(TestUvpDetector, InvalidReadBetweenTwoLowsKeepsCounting)
{
  Create();
  Feed(30.0f);
  Feed(10.0f);
  EXPECT_EQ(UvpState::OK, detector_->GetState());
  Feed(30.0f);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
  EXPECT_EQ(1u, detector_->GetInvalidReads(0));
}

TEST_F(TestUvpDetector, InvalidReadsLongerThanTheBlindTimeoutRestartTheCount)
{
  Create();
  Feed(30.0f);
  Feed(30.0f, 0, false);
  Feed(10.0f);
  Feed(10.0f);
  Feed(30.0f);
  EXPECT_EQ(UvpState::OK, detector_->GetState());
  Feed(30.0f);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
}

TEST_F(TestUvpDetector, ImplausibleReadsNeverTrip)
{
  Create();
  Feed(35.0f);
  for (int i = 0; i < 10; ++i) {
    // A missing pack reads below the plausible floor, an open thermistor fails the temperature.
    Feed(5.0f);
    Feed(19.9f);
    Feed(30.0f, 0, false);
    Feed(std::nanf(""));
    Feed(55.0f);
  }
  EXPECT_EQ(UvpState::OK, detector_->GetState());
  EXPECT_EQ(50u, detector_->GetInvalidReads(0));
}

TEST_F(TestUvpDetector, PlausibilityBoundsAreValid)
{
  Create(1, [] {
    auto config = Config();
    config.uvp_threshold = 28.0f;
    return config;
  }());
  Feed(20.0f);
  Feed(20.0f);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
  EXPECT_EQ(0u, detector_->GetInvalidReads(0));
}

TEST_F(TestUvpDetector, ThresholdsAreIndependent)
{
  Create();
  // Below ASAP only.
  Feed(31.2f);
  Feed(31.2f);
  EXPECT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());
  // One read below UVP does not trip it, even though the ASAP count goes on.
  Feed(30.9f);
  Feed(31.2f);
  EXPECT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());
  Feed(30.9f);
  Feed(30.9f);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
}

TEST_F(TestUvpDetector, UvpTripsStraightFromOk)
{
  Create();
  Feed(35.0f);
  Feed(29.0f);
  Feed(29.0f);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
}

TEST_F(TestUvpDetector, RecoveryAboveThresholdDoesNotClearUvp)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  ASSERT_EQ(UvpState::UVP, detector_->GetState());
  for (int i = 0; i < 20; ++i) {
    Feed(40.0f);
  }
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
}

TEST_F(TestUvpDetector, RecoveryAboveThresholdDoesNotClearChargeAsap)
{
  Create();
  Feed(31.2f);
  Feed(31.2f);
  ASSERT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());
  for (int i = 0; i < 20; ++i) {
    Feed(40.0f);
  }
  EXPECT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());
}

TEST_F(TestUvpDetector, DebouncedChargingClearsChargeAsap)
{
  Create();
  Feed(31.2f);
  Feed(31.2f);
  ASSERT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());

  Charge(true, 4);
  EXPECT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());
  EXPECT_FALSE(detector_->IsCharging());
  Charge(true, 1);
  EXPECT_TRUE(detector_->IsCharging());
  EXPECT_EQ(UvpState::OK, detector_->GetState());
}

TEST_F(TestUvpDetector, ChargingFlapRestartsTheDebounce)
{
  Create();
  Feed(31.2f);
  Feed(31.2f);
  ASSERT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());

  Charge(true, 4);
  Charge(false, 0);
  Charge(true, 4);
  EXPECT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());
  Charge(true, 1);
  EXPECT_EQ(UvpState::OK, detector_->GetState());
}

TEST_F(TestUvpDetector, ChargeAsapDoesNotTripWhileCharging)
{
  Create();
  Feed(35.0f);
  Charge(true, 5);
  Feed(31.2f);
  Feed(31.2f);
  EXPECT_EQ(UvpState::OK, detector_->GetState());

  // The reads were below the threshold all along, so the end of the charging trips it.
  Charge(false, 0);
  EXPECT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());
}

TEST_F(TestUvpDetector, ChargingBelowHysteresisKeepsUvp)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  ASSERT_EQ(UvpState::UVP, detector_->GetState());

  Charge(true, 5);
  Feed(31.4f);
  Charge(true, 1);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
}

TEST_F(TestUvpDetector, ChargingAtHysteresisClearsUvp)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  ASSERT_EQ(UvpState::UVP, detector_->GetState());

  // Above the threshold plus hysteresis without charging: still latched.
  Feed(31.5f);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());

  Charge(true, 5);
  EXPECT_EQ(UvpState::OK, detector_->GetState());
  EXPECT_EQ(1u, detector_->GetTripId());
}

TEST_F(TestUvpDetector, ChargingClearsUvpOnceTheVoltageRises)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  Charge(true, 5);
  ASSERT_EQ(UvpState::UVP, detector_->GetState());
  detector_->FeedCharging(0, true);
  Feed(32.0f);
  EXPECT_EQ(UvpState::OK, detector_->GetState());
}

TEST_F(TestUvpDetector, UvpTripsWhileCharging)
{
  Create();
  Feed(35.0f);
  Charge(true, 5);
  Feed(30.0f);
  Feed(30.0f);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
}

TEST_F(TestUvpDetector, EachTripIncrementsTheTripId)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  const auto first_stamp = detector_->GetTripStampNs();
  Feed(32.0f);
  Charge(true, 5);
  ASSERT_EQ(UvpState::OK, detector_->GetState());
  Charge(false, 0);
  Feed(30.0f);
  Feed(30.0f);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
  EXPECT_EQ(2u, detector_->GetTripId());
  EXPECT_GT(detector_->GetTripStampNs(), first_stamp);
}

TEST_F(TestUvpDetector, TripCurrentIsTheCurrentOfTheTripRead)
{
  Create();
  Feed(30.0f);
  detector_->FeedCurrent(0, 3.0f);
  Feed(30.0f);
  EXPECT_FLOAT_EQ(3.0f, detector_->GetTripCurrent(0));
  detector_->FeedCurrent(0, 12.5f);
  EXPECT_FLOAT_EQ(12.5f, detector_->GetTripCurrent(0));
  detector_->FeedCurrent(0, 1.0f);
  EXPECT_FLOAT_EQ(12.5f, detector_->GetTripCurrent(0));
}

TEST_F(TestUvpDetector, SecondsSinceTripOnBoottime)
{
  Create();
  EXPECT_TRUE(std::isnan(detector_->GetSecondsSinceTrip()));
  Feed(30.0f);
  Feed(30.0f);
  Advance(1500);
  EXPECT_NEAR(1.5, detector_->GetSecondsSinceTrip(), 0.01);
}

TEST_F(TestUvpDetector, BlindAfterTimeoutWithoutValidRead)
{
  Create();
  Feed(35.0f);
  EXPECT_FALSE(detector_->IsBlind());
  Feed(5.0f);
  Feed(5.0f);
  Feed(5.0f);
  EXPECT_TRUE(detector_->IsBlind());
  Feed(35.0f);
  EXPECT_FALSE(detector_->IsBlind());
}

TEST_F(TestUvpDetector, BlindKeepsTheLatch)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  Advance(10);
  EXPECT_TRUE(detector_->IsBlind());
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
}

TEST_F(TestUvpDetector, DualOneBatteryInUvpIsRobotUvp)
{
  Create(2);
  Feed(35.0f, 0);
  Feed(30.0f, 1);
  Feed(35.0f, 0);
  Feed(30.0f, 1);
  EXPECT_EQ(UvpState::OK, detector_->GetBatteryState(0));
  EXPECT_EQ(UvpState::UVP, detector_->GetBatteryState(1));
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
  EXPECT_TRUE(std::isnan(detector_->GetTripVoltage(0)));
  EXPECT_FLOAT_EQ(30.0f, detector_->GetTripVoltage(1));

  // The second battery tripping later is no new robot trip.
  Feed(30.0f, 0);
  Feed(30.0f, 0);
  EXPECT_EQ(UvpState::UVP, detector_->GetBatteryState(0));
  EXPECT_EQ(1u, detector_->GetTripId());
}

TEST_F(TestUvpDetector, DualRobotUvpClearsWhenEveryBatteryClears)
{
  Create(2);
  Feed(30.0f, 0);
  Feed(30.0f, 0);
  Feed(30.0f, 1);
  Feed(30.0f, 1);
  Feed(32.0f, 0);
  Charge(true, 5, 0);
  EXPECT_EQ(UvpState::OK, detector_->GetBatteryState(0));
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
  EXPECT_FALSE(detector_->IsCharging());
  Feed(32.0f, 1);
  Charge(true, 5, 1);
  EXPECT_EQ(UvpState::OK, detector_->GetState());
  EXPECT_TRUE(detector_->IsCharging());
}

TEST_F(TestUvpDetector, LatchRoundTrip)
{
  Create(2);
  Feed(30.0f, 1);
  detector_->FeedCurrent(1, 8.0f);
  Feed(29.5f, 1);
  detector_->FeedCurrent(1, 9.0f);
  ASSERT_EQ(UvpState::UVP, detector_->GetState());
  Advance(2000);

  const auto text = husarion_ugv_battery::SerializeUvpLatch(detector_->GetLatch("boot-a"));
  const auto latch = husarion_ugv_battery::ParseUvpLatch(text);

  // A new detector, as after a driver restart in the same boot.
  const auto trip_stamp = detector_->GetTripStampNs();
  Create(2);
  std::string why;
  ASSERT_TRUE(detector_->RestoreLatch(latch, "boot-a", why)) << why;
  EXPECT_EQ(UvpState::UVP, detector_->GetState());
  EXPECT_EQ(UvpState::UNKNOWN, detector_->GetBatteryState(0));
  EXPECT_EQ(UvpState::UVP, detector_->GetBatteryState(1));
  EXPECT_EQ(1u, detector_->GetTripId());
  EXPECT_EQ(trip_stamp, detector_->GetTripStampNs());
  EXPECT_FLOAT_EQ(29.5f, detector_->GetTripVoltage(1));
  EXPECT_FLOAT_EQ(9.0f, detector_->GetTripCurrent(1));
  EXPECT_TRUE(std::isnan(detector_->GetTripVoltage(0)));
  EXPECT_NEAR(2.0, detector_->GetSecondsSinceTrip(), 0.01);

  // High voltage after the restart does not clear it.
  Feed(40.0f, 1);
  Feed(40.0f, 1);
  EXPECT_EQ(UvpState::UVP, detector_->GetState());

  // The next trip continues the numbering.
  Feed(40.0f, 0);
  Charge(true, 5, 1);
  ASSERT_EQ(UvpState::OK, detector_->GetState());
  Charge(false, 0, 1);
  Feed(30.0f, 0);
  Feed(30.0f, 0);
  EXPECT_EQ(2u, detector_->GetTripId());
}

TEST_F(TestUvpDetector, LatchFromAnotherBootIsRefused)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  const auto latch = detector_->GetLatch("boot-a");

  Create();
  std::string why;
  EXPECT_FALSE(detector_->RestoreLatch(latch, "boot-b", why));
  EXPECT_FALSE(why.empty());
  EXPECT_EQ(UvpState::UNKNOWN, detector_->GetState());
  EXPECT_EQ(0u, detector_->GetTripId());
}

TEST_F(TestUvpDetector, LatchFromBeforeAReboot)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  const auto latch = detector_->GetLatch("boot-a");

  // CLOCK_BOOTTIME started again, with the same boot id.
  boottime_ns_ = 5 * kMs;
  Create();
  std::string why;
  EXPECT_FALSE(detector_->RestoreLatch(latch, "boot-a", why));
  EXPECT_EQ(UvpState::UNKNOWN, detector_->GetState());
}

TEST_F(TestUvpDetector, LatchWithAnUnknownBootIdIsRefused)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  const auto latch = detector_->GetLatch("");
  Advance(1000);

  Create();
  std::string why;
  EXPECT_FALSE(detector_->RestoreLatch(latch, "", why));
  EXPECT_EQ(UvpState::UNKNOWN, detector_->GetState());
}

TEST_F(TestUvpDetector, TimeWithoutChargingStartsAtTheTrip)
{
  Create();
  EXPECT_TRUE(std::isnan(detector_->GetSecondsWithoutCharging()));
  Feed(30.0f);
  Feed(30.0f);
  Advance(1500);
  EXPECT_NEAR(1.5, detector_->GetSecondsWithoutCharging(), 0.01);
}

TEST_F(TestUvpDetector, TimeWithoutChargingStartsAgainWhenTheChargeEnds)
{
  Create();
  Feed(30.0f);
  Feed(30.0f);
  Advance(3000);
  const auto latch_revision = detector_->GetLatchRevision();

  Charge(true, 5);
  ASSERT_TRUE(detector_->IsCharging());
  EXPECT_TRUE(std::isnan(detector_->GetSecondsWithoutCharging()));
  ASSERT_EQ(UvpState::UVP, detector_->GetState());

  Charge(false, 0);
  Advance(200);
  EXPECT_NEAR(0.2, detector_->GetSecondsWithoutCharging(), 0.01);
  EXPECT_NEAR(3.2, detector_->GetSecondsSinceTrip(), 0.02);
  // Written to the latch, so a driver restart keeps it.
  EXPECT_EQ(latch_revision + 1, detector_->GetLatchRevision());

  const auto latch = husarion_ugv_battery::ParseUvpLatch(
    husarion_ugv_battery::SerializeUvpLatch(detector_->GetLatch("boot-a")));
  Create();
  std::string why;
  ASSERT_TRUE(detector_->RestoreLatch(latch, "boot-a", why)) << why;
  EXPECT_NEAR(0.2, detector_->GetSecondsWithoutCharging(), 0.01);
}

TEST_F(TestUvpDetector, ChargerConnectedIsNotDebounced)
{
  Create();
  EXPECT_FALSE(detector_->IsChargerConnected());
  detector_->FeedCharging(0, true);
  EXPECT_TRUE(detector_->IsChargerConnected());
  EXPECT_FALSE(detector_->IsCharging());
  detector_->FeedCharging(0, false);
  EXPECT_FALSE(detector_->IsChargerConnected());
}

TEST_F(TestUvpDetector, LatchForAnotherBatteryCountIsRefused)
{
  Create(2);
  const auto latch = detector_->GetLatch("boot-a");
  Create(1);
  std::string why;
  EXPECT_FALSE(detector_->RestoreLatch(latch, "boot-a", why));
}

TEST_F(TestUvpDetector, MalformedLatchThrows)
{
  EXPECT_THROW(husarion_ugv_battery::ParseUvpLatch("garbage"), std::runtime_error);
  EXPECT_THROW(husarion_ugv_battery::ParseUvpLatch("boot_id=a\ntrip_id=x\n"), std::runtime_error);
  EXPECT_THROW(
    husarion_ugv_battery::ParseUvpLatch(
      "boot_id=a\ntrip_id=1\ntrip_boottime_ns=1\ntrip_stamp_ns=1\ncountdown_boottime_ns=1\n"
      "battery_count=1\n"
      "battery_0=MAYBE,nan,nan\n"),
    std::runtime_error);
  // Every key is required, the start of the time without charging included.
  EXPECT_THROW(
    husarion_ugv_battery::ParseUvpLatch(
      "boot_id=a\ntrip_id=1\ntrip_boottime_ns=7\ntrip_stamp_ns=1\nbattery_count=1\n"
      "battery_0=UVP,30,1\n"),
    std::runtime_error);
}

TEST_F(TestUvpDetector, LatchRevisionCountsUvpTransitionsOnly)
{
  Create();
  const auto start = detector_->GetLatchRevision();
  Feed(31.2f);
  Feed(31.2f);
  ASSERT_EQ(UvpState::CHARGE_ASAP, detector_->GetState());
  EXPECT_EQ(start, detector_->GetLatchRevision());
  Feed(30.0f);
  Feed(30.0f);
  EXPECT_EQ(start + 1, detector_->GetLatchRevision());
  Feed(32.0f);
  Charge(true, 5);
  EXPECT_EQ(start + 2, detector_->GetLatchRevision());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
