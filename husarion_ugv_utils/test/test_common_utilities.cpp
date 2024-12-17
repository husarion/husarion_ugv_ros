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

#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

#include "husarion_ugv_utils/common_utilities.hpp"

TEST(TestSetPrecision, TwoDigitPrecision)
{
  float value = 3.14159;
  float expected_result = 3.14;

  float result = husarion_ugv_utils::common_utilities::SetPrecision(value, 2);

  EXPECT_FLOAT_EQ(expected_result, result);
}

TEST(TestSetPrecision, ZeroDigitPrecision)
{
  float value = 3.54159;
  float expected_result = 4.0;

  float result = husarion_ugv_utils::common_utilities::SetPrecision(value, 0);

  EXPECT_FLOAT_EQ(expected_result, result);
}

TEST(TestSetPrecision, NegativeValue)
{
  float value = -3.14159;
  float expected_result = -3.14;
  float result = husarion_ugv_utils::common_utilities::SetPrecision(value, 2);
  EXPECT_FLOAT_EQ(expected_result, result);
}

TEST(TestSetPrecision, LargeValue)
{
  float value = 123456.789;
  float expected_result = 123456.79;
  float result = husarion_ugv_utils::common_utilities::SetPrecision(value, 2);
  EXPECT_FLOAT_EQ(expected_result, result);
}

TEST(TestCountPercentage, ValidValues)
{
  int value = 25;
  int total = 100;
  float expected_result = 25.00;

  float result = husarion_ugv_utils::common_utilities::CountPercentage(value, total);

  EXPECT_FLOAT_EQ(expected_result, result);
}

TEST(TestCountPercentage, ZeroTotal)
{
  int value = 25;
  int total = 0;

  EXPECT_THROW(
    husarion_ugv_utils::common_utilities::CountPercentage(value, total), std::invalid_argument);
}

TEST(TestCountPercentage, ZeroValue)
{
  int value = 0;
  int total = 100;
  float expected_result = 0.00;

  float result = husarion_ugv_utils::common_utilities::CountPercentage(value, total);

  EXPECT_FLOAT_EQ(expected_result, result);
}

TEST(TestPrefixMapKeys, CorrectlyPrefixesKeys)
{
  std::map<std::string, int> input_map = {{"key1", 1}, {"key2", 2}};
  std::string prefix = "prefix_";
  std::map<std::string, int> expected_map = {{"prefix_key1", 1}, {"prefix_key2", 2}};

  auto result_map = husarion_ugv_utils::common_utilities::PrefixMapKeys(input_map, prefix);

  EXPECT_EQ(result_map, expected_map);
}

TEST(TestPrefixMapKeys, HandlesEmptyPrefix)
{
  std::map<std::string, int> input_map = {{"key1", 1}, {"key2", 2}};
  std::string prefix = "";
  std::map<std::string, int> expected_map = input_map;  // The maps should be identical

  auto result_map = husarion_ugv_utils::common_utilities::PrefixMapKeys(input_map, prefix);

  EXPECT_EQ(result_map, expected_map);
}

TEST(TestPrefixMapKeys, HandlesEmptyMap)
{
  std::map<std::string, int> input_map;
  std::string prefix = "prefix_";
  std::map<std::string, int> expected_map;

  auto result_map = husarion_ugv_utils::common_utilities::PrefixMapKeys(input_map, prefix);

  EXPECT_EQ(result_map, expected_map);
}

TEST(TestPrefixMapKeys, HandlesNonAlphanumericPrefix)
{
  std::map<std::string, int> input_map = {{"key1", 1}, {"key2", 2}};
  std::string prefix = "prefix_@#";
  std::map<std::string, int> expected_map = {{"prefix_@#key1", 1}, {"prefix_@#key2", 2}};

  auto result_map = husarion_ugv_utils::common_utilities::PrefixMapKeys(input_map, prefix);

  EXPECT_EQ(result_map, expected_map);
}

TEST(TestOpenFile, HandleOpenFile)
{
  std::string path = testing::TempDir() + "test_husarion_ugv_utils_open_file";

  // Make sure that there is no random file.
  std::filesystem::remove(path);

  std::ofstream ofs(path);
  ofs.close();

  EXPECT_NO_THROW({ husarion_ugv_utils::common_utilities::OpenFile(path, std::ios_base::out); });
  std::filesystem::remove(path);
}

TEST(TestOpenFile, HandleOpenFileThrow)
{
  std::string path = testing::TempDir() + "test_husarion_ugv_utils_open_file";

  // Make sure that there is no random file.
  std::filesystem::remove(path);

  EXPECT_THROW(
    { husarion_ugv_utils::common_utilities::OpenFile(path, std::ios_base::in); },
    std::runtime_error);
}

TEST(TestMeetsVersionRequirement, SameVersion)
{
  float version = 1.06;

  auto is_met = husarion_ugv_utils::common_utilities::MeetsVersionRequirement(version, 1.06);

  EXPECT_TRUE(is_met);
}

TEST(TestMeetsVersionRequirement, LowerVersion)
{
  float version = 1.2;

  auto is_met = husarion_ugv_utils::common_utilities::MeetsVersionRequirement(version, 1.06);

  EXPECT_TRUE(is_met);
}

TEST(TestMeetsVersionRequirement, HigherVersion)
{
  float version = 1.0;

  auto is_met = husarion_ugv_utils::common_utilities::MeetsVersionRequirement(version, 1.06);

  EXPECT_FALSE(is_met);
}

TEST(TestMeetsVersionRequirement, NaNVersionRequired)
{
  float version = std::numeric_limits<float>::quiet_NaN();

  auto is_met = husarion_ugv_utils::common_utilities::MeetsVersionRequirement(version, 1.06);

  EXPECT_FALSE(is_met);
}

TEST(TestMeetsVersionRequirement, CorrectlyComparesVersions)
{
  float panther_version = 1.06;
  EXPECT_TRUE(husarion_ugv_utils::common_utilities::MeetsVersionRequirement(panther_version, 0.0));
  EXPECT_TRUE(husarion_ugv_utils::common_utilities::MeetsVersionRequirement(panther_version, 1.0));
  EXPECT_TRUE(husarion_ugv_utils::common_utilities::MeetsVersionRequirement(panther_version, 1.06));
  EXPECT_FALSE(husarion_ugv_utils::common_utilities::MeetsVersionRequirement(
    panther_version, std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(husarion_ugv_utils::common_utilities::MeetsVersionRequirement(panther_version, 1.2));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
