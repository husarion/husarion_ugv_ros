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

#include "gtest/gtest.h"

#include "husarion_ugv_manager/charge_asap_indicator.hpp"

using husarion_ugv_manager::ChargeAsapIndicator;

namespace
{
constexpr std::int64_t kMs = 1000000;
constexpr std::uint8_t kStateOk = 1;
constexpr std::uint8_t kStateUvp = 3;
}  // namespace

TEST(TestChargeAsapIndicator, NothingWithoutData)
{
  ChargeAsapIndicator indicator;
  EXPECT_FALSE(indicator.Update(0));
}

TEST(TestChargeAsapIndicator, PlaysOnlyWithReactionsEnabled)
{
  ChargeAsapIndicator indicator;
  indicator.SetProtection(ChargeAsapIndicator::kStateChargeAsap, false, 0);
  EXPECT_FALSE(indicator.Update(10 * kMs));

  indicator.SetProtection(ChargeAsapIndicator::kStateChargeAsap, true, 20 * kMs);
  EXPECT_TRUE(indicator.Update(30 * kMs));
}

TEST(TestChargeAsapIndicator, OnlyInChargeAsap)
{
  ChargeAsapIndicator indicator;
  for (const auto state : {kStateOk, kStateUvp}) {
    indicator.SetProtection(state, true, 0);
    EXPECT_FALSE(indicator.Update(10 * kMs));
  }
}

TEST(TestChargeAsapIndicator, RepeatsEveryPeriod)
{
  ChargeAsapIndicator indicator;
  int sent = 0;
  for (std::int64_t t = 0; t < 20000 * kMs; t += 200 * kMs) {
    indicator.SetProtection(ChargeAsapIndicator::kStateChargeAsap, true, t);
    sent += indicator.Update(t) ? 1 : 0;
  }
  // At 0, 5, 10 and 15 s.
  EXPECT_EQ(4, sent);
}

TEST(TestChargeAsapIndicator, StopsWhenCleared)
{
  ChargeAsapIndicator indicator;
  indicator.SetProtection(ChargeAsapIndicator::kStateChargeAsap, true, 0);
  EXPECT_TRUE(indicator.Update(0));
  indicator.SetProtection(kStateOk, true, 100 * kMs);
  EXPECT_FALSE(indicator.Update(200 * kMs));

  // Back in CHARGE_ASAP: shown at once, not a period after the last one.
  indicator.SetProtection(ChargeAsapIndicator::kStateChargeAsap, true, 300 * kMs);
  EXPECT_TRUE(indicator.Update(300 * kMs));
}

TEST(TestChargeAsapIndicator, StopsOnStaleData)
{
  ChargeAsapIndicator indicator;
  indicator.SetProtection(ChargeAsapIndicator::kStateChargeAsap, true, 0);
  EXPECT_TRUE(indicator.Update(0));
  EXPECT_FALSE(indicator.Update(5000 * kMs));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
