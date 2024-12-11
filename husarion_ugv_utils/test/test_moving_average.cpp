// Copyright 2024 Husarion sp. z o.o.
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

#include "gtest/gtest.h"

#include "husarion_ugv_utils/moving_average.hpp"

TEST(TestMovingAverage, TestDefaultInitialValue)
{
  husarion_ugv_utils::MovingAverage<double> ma(4);
  EXPECT_EQ(0.0, ma.GetAverage());
}

TEST(TestMovingAverage, TestIntialValue)
{
  husarion_ugv_utils::MovingAverage<float> ma(4, 1.0);
  EXPECT_EQ(1.0, ma.GetAverage());

  husarion_ugv_utils::MovingAverage<double> ma_1(10, 3.7);
  EXPECT_EQ(3.7, ma_1.GetAverage());

  husarion_ugv_utils::MovingAverage<int> ma_2(3, 4);
  EXPECT_EQ(4, ma_2.GetAverage());
}

TEST(TestMovingAverage, TestOutputValues)
{
  husarion_ugv_utils::MovingAverage<double> ma(4);
  ma.Roll(1.0);
  ASSERT_EQ(1.0, ma.GetAverage());

  ma.Roll(2.0);
  ASSERT_EQ(1.5, ma.GetAverage());

  ma.Roll(3.0);
  ma.Roll(4.0);
  ASSERT_EQ(2.5, ma.GetAverage());

  ma.Roll(5.0);
  ma.Roll(5.0);
  ma.Roll(5.0);
  ma.Roll(5.0);
  ASSERT_EQ(5.0, ma.GetAverage());
}

TEST(TestMovingAverage, TestHighOverload)
{
  const std::size_t window_len = 1000;
  husarion_ugv_utils::MovingAverage<double> ma(window_len);

  double sum = 0.0;
  for (std::size_t i = 1; i <= window_len * 10; i++) {
    sum += double(i);
    ma.Roll(double(i));

    // test every 1000 rolls expected average
    if (i % window_len == 0) {
      ASSERT_NEAR(
        sum / double(window_len), ma.GetAverage(), std::numeric_limits<double>::epsilon());
      sum = 0.0;
    }
  }
}

TEST(TestMovingAverage, TestIntFloorRound)
{
  husarion_ugv_utils::MovingAverage<int> ma;
  ma.Roll(1);
  ma.Roll(2);
  EXPECT_EQ(1, ma.GetAverage());
}

TEST(TestMovingAverage, TestReset)
{
  husarion_ugv_utils::MovingAverage<double> ma(4);
  ma.Roll(1.0);
  ma.Roll(2.0);
  ma.Roll(1.0);
  ma.Roll(2.0);
  EXPECT_EQ(1.5, ma.GetAverage());

  ma.Reset();
  EXPECT_EQ(0.0, ma.GetAverage());

  ma.Roll(2.0);
  ma.Roll(4.0);
  EXPECT_EQ(3.0, ma.GetAverage());
}

TEST(TestMovingAverage, TestResetToInitialValue)
{
  husarion_ugv_utils::MovingAverage<double> ma(4, 7.0);
  ma.Roll(1.0);
  ma.Roll(2.0);
  EXPECT_EQ(1.5, ma.GetAverage());
  ma.Reset();
  EXPECT_EQ(7.0, ma.GetAverage());
}

TEST(TestMovingAverage, TestInfInjectionHandling)
{
  husarion_ugv_utils::MovingAverage<double> ma(4);
  ma.Roll(1.0);
  ma.Roll(2.0);
  ma.Roll(3.0);
  ma.Roll(4.0);
  EXPECT_EQ(2.5, ma.GetAverage());

  ma.Roll(std::numeric_limits<double>::infinity());
  EXPECT_EQ(std::numeric_limits<double>::infinity(), ma.GetAverage());

  ma.Roll(1.0);
  ma.Roll(2.0);
  ma.Roll(3.0);
  ma.Roll(4.0);
  EXPECT_EQ(2.5, ma.GetAverage());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
