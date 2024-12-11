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

#include <map>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

#include "husarion_ugv_utils/diagnostics.hpp"

TEST(TestAddKeyValueIfTrue, HandlesBoolean)
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  std::map<std::string, bool> kv_map = {{"key1", true}, {"key2", false}, {"key3", true}};

  husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(status, kv_map);

  ASSERT_EQ(2, status.values.size());
  EXPECT_EQ("key1", status.values.at(0).key);
  EXPECT_EQ("key3", status.values.at(1).key);
}

TEST(TestAddKeyValueIfTrue, HandlesSmartPointer)
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  std::map<std::string, std::shared_ptr<int>> kv_map = {
    {"key1", nullptr}, {"key2", nullptr}, {"key3", std::make_shared<int>(1)}};

  husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(status, kv_map);

  ASSERT_EQ(1, status.values.size());
  EXPECT_EQ("key3", status.values.at(0).key);
}

TEST(TestAddKeyValueIfTrue, HandlesInteger)
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  std::map<std::string, int> kv_map = {{"key1", 0}, {"key2", 1}, {"key3", 2}};

  husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(status, kv_map);

  ASSERT_EQ(2, status.values.size());
  EXPECT_EQ("key2", status.values.at(0).key);
  EXPECT_EQ("key3", status.values.at(1).key);
}

TEST(TestAddKeyValueIfTrue, HandlesEmptyMap)
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  std::map<std::string, bool> kv_map;

  husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(status, kv_map);

  ASSERT_EQ(0, status.values.size());
}

TEST(TestAddKeyValueIfTrue, HandlesPrefixAndSuffix)
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  std::map<std::string, bool> kv_map = {{"key1", true}, {"key2", false}, {"key3", true}};

  husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(status, kv_map, "prefix_", "_suffix");

  ASSERT_EQ(2, status.values.size());
  EXPECT_EQ("prefix_key1_suffix", status.values.at(0).key);
  EXPECT_EQ("prefix_key3_suffix", status.values.at(1).key);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
