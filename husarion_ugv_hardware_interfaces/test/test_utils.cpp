// Copyright 2023 Husarion sp. z o.o.
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

#include <bitset>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <husarion_ugv_hardware_interfaces/utils.hpp>

TEST(TestUtils, GetByte)
{
  using husarion_ugv_hardware_interfaces::GetByte;

  EXPECT_EQ(GetByte(static_cast<std::int32_t>(0xFA3B4186), 0), 0x86);
  EXPECT_EQ(GetByte(static_cast<std::int32_t>(0xFA3B4186), 1), 0x41);
  EXPECT_EQ(GetByte(static_cast<std::int32_t>(0xFA3B4186), 2), 0x3B);
  EXPECT_EQ(GetByte(static_cast<std::int32_t>(0xFA3B4186), 3), 0xFA);
}

TEST(TestUtils, GetByteOutOfRange)
{
  using husarion_ugv_hardware_interfaces::GetByte;

  EXPECT_THROW(GetByte(static_cast<std::int32_t>(0xFA3B4186), 4), std::runtime_error);
  EXPECT_THROW(GetByte(static_cast<std::int32_t>(0xFA3B4186), -1), std::runtime_error);
}

TEST(TestUtils, OperationWithAttemptsFailTest)
{
  unsigned max_attempts = 5;
  unsigned attempts_counter = 0;
  unsigned on_error_counter = 0;

  EXPECT_FALSE(husarion_ugv_hardware_interfaces::OperationWithAttempts(
    [&attempts_counter]() {
      ++attempts_counter;
      throw std::runtime_error("");
    },
    max_attempts, [&on_error_counter]() { ++on_error_counter; }));
  EXPECT_EQ(attempts_counter, max_attempts);
  EXPECT_EQ(on_error_counter, max_attempts);
}

TEST(TestUtils, OperationWithAttemptsSuccessTest)
{
  unsigned max_attempts = 5;
  unsigned attempts_counter = 0;

  EXPECT_TRUE(husarion_ugv_hardware_interfaces::OperationWithAttempts(
    [&attempts_counter, &max_attempts]() {
      ++attempts_counter;
      if (attempts_counter < max_attempts) {
        throw std::runtime_error("");
      }
    },
    max_attempts, []() {}));
  EXPECT_EQ(attempts_counter, max_attempts);
}

TEST(TestUtils, OperationWithAttemptsOnErrorThrowTest)
{
  EXPECT_FALSE(husarion_ugv_hardware_interfaces::OperationWithAttempts(
    []() { throw std::runtime_error(""); }, 5, []() { throw std::runtime_error(""); }));
}

TEST(TestUtils, CheckIfJointNameContainValidSequence)
{
  using husarion_ugv_hardware_interfaces::CheckIfJointNameContainValidSequence;

  EXPECT_TRUE(CheckIfJointNameContainValidSequence("fr_wheel_joint", "fr"));
  EXPECT_TRUE(CheckIfJointNameContainValidSequence("namespace/fr_wheel_joint", "fr"));
  EXPECT_TRUE(CheckIfJointNameContainValidSequence("wheel_fr_joint", "fr"));
  EXPECT_TRUE(CheckIfJointNameContainValidSequence("wheel_joint_fr", "fr"));

  EXPECT_FALSE(CheckIfJointNameContainValidSequence("wheel_joint", "fr"));
  EXPECT_FALSE(CheckIfJointNameContainValidSequence("rfr_wheel_joint", "fr"));
  EXPECT_FALSE(CheckIfJointNameContainValidSequence("frwheel_joint", "fr"));
  EXPECT_FALSE(CheckIfJointNameContainValidSequence("wheel_froint", "fr"));
  EXPECT_FALSE(CheckIfJointNameContainValidSequence("wheeljointfr", "fr"));
}

TEST(TestUtils, ParseRTPriority)
{
  using husarion_ugv_hardware_interfaces::ParseRTPriority;

  EXPECT_EQ(ParseRTPriority("1"), 1u);
  EXPECT_EQ(ParseRTPriority("70"), 70u);
  EXPECT_EQ(ParseRTPriority("98"), 98u);
  EXPECT_EQ(ParseRTPriority("065"), 65u);

  EXPECT_FALSE(ParseRTPriority(""));
  EXPECT_FALSE(ParseRTPriority("0"));
  EXPECT_FALSE(ParseRTPriority("99"));
  EXPECT_FALSE(ParseRTPriority("-5"));
  EXPECT_FALSE(ParseRTPriority("+5"));
  EXPECT_FALSE(ParseRTPriority(" 60"));
  EXPECT_FALSE(ParseRTPriority("60.5"));
  EXPECT_FALSE(ParseRTPriority("high"));
  EXPECT_FALSE(ParseRTPriority("0070"));
  EXPECT_FALSE(ParseRTPriority("99999999999999999999"));
}

TEST(TestUtils, ParseCPU)
{
  using husarion_ugv_hardware_interfaces::ParseCPU;

  EXPECT_EQ(ParseCPU("-1", 4), -1);
  EXPECT_EQ(ParseCPU("0", 4), 0);
  EXPECT_EQ(ParseCPU("3", 4), 3);

  EXPECT_FALSE(ParseCPU("4", 4));
  EXPECT_FALSE(ParseCPU("3", 2));
  EXPECT_FALSE(ParseCPU("", 4));
  EXPECT_FALSE(ParseCPU("-2", 4));
  EXPECT_FALSE(ParseCPU(" 1", 4));
  EXPECT_FALSE(ParseCPU("1.0", 4));
  EXPECT_FALSE(ParseCPU("cpu0", 4));
  EXPECT_FALSE(ParseCPU("99999999999", 4));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
