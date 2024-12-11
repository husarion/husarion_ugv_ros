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

#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "husarion_ugv_utils/test/test_utils.hpp"

template <typename T>
void TestCheckNaNVector()
{
  std::vector<T> vector(10, std::numeric_limits<T>::quiet_NaN());
  EXPECT_TRUE(husarion_ugv_utils::test_utils::CheckNaNVector(vector));
  vector.push_back(1.0);
  EXPECT_FALSE(husarion_ugv_utils::test_utils::CheckNaNVector(vector));
}

TEST(TestTestUtils, CheckNanVector)
{
  TestCheckNaNVector<float>();
  TestCheckNaNVector<double>();
  TestCheckNaNVector<long double>();
  EXPECT_THROW(TestCheckNaNVector<int>(), std::runtime_error);
}

TEST(TestTestUtils, IsMessageThrownTrue)
{
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    []() { throw std::runtime_error("Example exception"); }, "Example exception"));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::out_of_range>(
    []() { throw std::out_of_range("Example exception"); }, "Example exception"));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::invalid_argument>(
    []() { throw std::invalid_argument("Example exception"); }, "Example exception"));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    []() { throw std::runtime_error("Example exception"); }, "Example"));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    []() { throw std::runtime_error("Example exception"); }, "exception"));
}

TEST(TestTestUtils, IsMessageThrownDifferentException)
{
  EXPECT_FALSE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    []() { throw std::out_of_range("Example exception"); }, "Example exception"));

  EXPECT_FALSE(husarion_ugv_utils::test_utils::IsMessageThrown<std::out_of_range>(
    []() { throw std::invalid_argument("Example exception"); }, "Example exception"));

  EXPECT_FALSE(husarion_ugv_utils::test_utils::IsMessageThrown<std::invalid_argument>(
    []() { throw std::runtime_error("Example exception"); }, "Example exception"));
}

TEST(TestTestUtils, IsMessageThrownDifferentMessage)
{
  EXPECT_FALSE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    []() { throw std::runtime_error("Example exception"); }, "Different exception message"));

  EXPECT_FALSE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    []() { throw std::runtime_error("Example exception"); }, "Example exception "));
}

TEST(TestTestUtils, IsMessageThrownNoThrow)
{
  EXPECT_FALSE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    []() { return; }, "Example exception"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
