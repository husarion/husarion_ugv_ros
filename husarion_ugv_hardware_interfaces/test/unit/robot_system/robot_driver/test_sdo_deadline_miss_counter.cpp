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

#include <chrono>

#include <gtest/gtest.h>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/sdo_deadline_miss_counter.hpp"

using husarion_ugv_hardware_interfaces::SdoDeadlineMissCounter;
using namespace std::chrono_literals;

class TestSdoDeadlineMissCounter : public testing::Test
{
protected:
  std::chrono::steady_clock::time_point now_{};
  SdoDeadlineMissCounter counter_{3, 60s, [this]() { return now_; }};
};

TEST_F(TestSdoDeadlineMissCounter, ThreeMissesInTheWindowTrip)
{
  EXPECT_FALSE(counter_.OnMiss());
  now_ += 20s;
  EXPECT_FALSE(counter_.OnMiss());
  now_ += 20s;
  EXPECT_TRUE(counter_.OnMiss());
}

TEST_F(TestSdoDeadlineMissCounter, DeliveredConfirmationClearsTheCount)
{
  EXPECT_FALSE(counter_.OnMiss());
  EXPECT_FALSE(counter_.OnMiss());
  counter_.OnDelivered();
  EXPECT_FALSE(counter_.OnMiss());
  EXPECT_FALSE(counter_.OnMiss());
  EXPECT_TRUE(counter_.OnMiss());
}

TEST_F(TestSdoDeadlineMissCounter, MissAfterTheWindowStartsANewCount)
{
  // The window is anchored at the first miss, so misses at 0, 59 and 61 s do not trip it.
  EXPECT_FALSE(counter_.OnMiss());
  now_ += 59s;
  EXPECT_FALSE(counter_.OnMiss());
  now_ += 2s;
  EXPECT_FALSE(counter_.OnMiss());
  EXPECT_FALSE(counter_.OnMiss());
  EXPECT_TRUE(counter_.OnMiss());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
