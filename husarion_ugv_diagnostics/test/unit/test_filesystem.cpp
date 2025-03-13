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

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "husarion_ugv_diagnostics/filesystem.hpp"

class TestFilesystem : public testing::Test
{
public:
  TestFilesystem();

  std::string CreateTestFile(const std::string & content);
  void RemoveTestFile(const std::string & file_path);

protected:
  std::shared_ptr<husarion_ugv_diagnostics::Filesystem> filesystem_;

  static constexpr char kDummyString[] = "Hello World!";
};

TestFilesystem::TestFilesystem()
: filesystem_(std::make_shared<husarion_ugv_diagnostics::Filesystem>())
{
}

std::string TestFilesystem::CreateTestFile(const std::string & content)
{
  const std::string file_path = testing::TempDir() + "test_file.txt";

  std::ofstream file(file_path);
  file << content;
  file.close();

  return file_path;
}

void TestFilesystem::RemoveTestFile(const std::string & file_path)
{
  std::remove(file_path.c_str());
}

TEST_F(TestFilesystem, ReadFileSuccess)
{
  const auto test_file_path = CreateTestFile(kDummyString);

  auto content = filesystem_->ReadFile(test_file_path);

  EXPECT_STREQ(kDummyString, content.c_str());

  RemoveTestFile(test_file_path);
}

TEST_F(TestFilesystem, ReadFileNonExistent)
{
  // File does not exist
  auto const test_file_path = testing::TempDir() + "test_missing_file.txt";

  EXPECT_THROW(filesystem_->ReadFile(test_file_path), std::invalid_argument);
}

TEST_F(TestFilesystem, ReadFileLocked)
{
  const auto test_file_path = CreateTestFile(kDummyString);

  // Change file permissions to make it unreadable
  std::filesystem::permissions(test_file_path, std::filesystem::perms::none);

  EXPECT_THROW(filesystem_->ReadFile(test_file_path), std::runtime_error);

  // Restore file permissions to allow deletion
  std::filesystem::permissions(test_file_path, std::filesystem::perms::owner_all);

  RemoveTestFile(test_file_path);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
