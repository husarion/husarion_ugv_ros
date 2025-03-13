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
#include "yaml-cpp/yaml.h"

#include "husarion_ugv_utils/test/test_utils.hpp"
#include "husarion_ugv_utils/yaml_utils.hpp"

TEST(TestGetYAMLKeyValue, MissingKey)
{
  YAML::Node desc;
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [desc]() { husarion_ugv_utils::GetYAMLKeyValue<YAML::Node>(desc, "key_name"); },
    "Missing 'key_name' in description"));
}

TEST(TestGetYAMLKeyValue, ConversionFailure)
{
  YAML::Node desc;

  desc["float_key"] = 1.5;
  desc["string_key"] = "string";
  desc["int_vector_key"] = "[1 2 3]";

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { husarion_ugv_utils::GetYAMLKeyValue<int>(desc, "float_key"); }, "Failed to convert"));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { husarion_ugv_utils::GetYAMLKeyValue<float>(desc, "string_key"); },
    "Failed to convert"));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { husarion_ugv_utils::GetYAMLKeyValue<int>(desc, "int_vector_key"); },
    "Failed to convert"));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { husarion_ugv_utils::GetYAMLKeyValue<std::vector<std::string>>(desc, "string_key"); },
    "Failed to convert"));
}

TEST(TestGetYAMLKeyValue, GetKey)
{
  YAML::Node desc;

  desc["int_key"] = 2;
  desc["float_key"] = 1.5;
  desc["string_key"] = "string";

  const auto int_value = husarion_ugv_utils::GetYAMLKeyValue<int>(desc, "int_key");
  EXPECT_EQ(2, int_value);

  const auto float_value = husarion_ugv_utils::GetYAMLKeyValue<float>(desc, "float_key");
  EXPECT_EQ(1.5, float_value);

  const auto str_value = husarion_ugv_utils::GetYAMLKeyValue<std::string>(desc, "string_key");
  EXPECT_EQ("string", str_value);
}

TEST(TestGetYAMLKeyValue, GetVectorKey)
{
  YAML::Node desc;

  desc["int_vector_key"] = std::vector<int>(5, 147);

  const auto value_vector = husarion_ugv_utils::GetYAMLKeyValue<std::vector<int>>(
    desc, "int_vector_key");
  for (auto value : value_vector) {
    EXPECT_EQ(147, value);
  }
}

TEST(TestGetYAMLKeyValue, GetKeyDefaultValue)
{
  YAML::Node desc;
  const int default_value = 54;

  const auto value = husarion_ugv_utils::GetYAMLKeyValue<int>(desc, "key_name", default_value);
  EXPECT_EQ(default_value, value);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
