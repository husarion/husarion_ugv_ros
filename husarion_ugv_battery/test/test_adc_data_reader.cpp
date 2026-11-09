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

#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "husarion_ugv_battery/adc_data_reader.hpp"

class TestADCDataReader : public testing::Test
{
public:
  TestADCDataReader();
  ~TestADCDataReader();

protected:
  std::shared_ptr<husarion_ugv_battery::ADCDataReader> data_reader_;
  std::filesystem::path data_file_path_;
  std::filesystem::path scale_file_path_;
  std::ofstream file_;

  template <typename T>
  void WriteNumberToFile(const T number, const std::string file_path);
};

TestADCDataReader::TestADCDataReader()
{
  std::filesystem::path current_path = testing::TempDir();
  data_file_path_ = current_path / "in_voltage0_raw";
  scale_file_path_ = current_path / "in_voltage0_scale";

  WriteNumberToFile<float>(1.0, scale_file_path_);

  data_reader_ = std::make_shared<husarion_ugv_battery::ADCDataReader>(current_path);
}

TestADCDataReader::~TestADCDataReader()
{
  std::filesystem::remove(data_file_path_);
  std::filesystem::remove(scale_file_path_);
}

template <typename T>
void TestADCDataReader::WriteNumberToFile(const T number, const std::string file_path)
{
  std::ofstream file(file_path);
  if (file.is_open()) {
    file << number;
    file.close();
  }
}

TEST_F(TestADCDataReader, TestGetADCMeasurement)
{
  WriteNumberToFile<int>(1420, data_file_path_);

  auto data = data_reader_->GetADCMeasurement(0, 0);
  EXPECT_FLOAT_EQ(1.42, data);

  WriteNumberToFile<int>(200, data_file_path_);
  data = data_reader_->GetADCMeasurement(0, 0);
  EXPECT_FLOAT_EQ(0.2, data);

  WriteNumberToFile<int>(-200, data_file_path_);
  data = data_reader_->GetADCMeasurement(0, 0);
  EXPECT_FLOAT_EQ(-0.2, data);

  WriteNumberToFile<int>(200, data_file_path_);
  data = data_reader_->GetADCMeasurement(0, 100);
  EXPECT_FLOAT_EQ(0.1, data);

  WriteNumberToFile<float>(0.5, scale_file_path_);
  data = data_reader_->GetADCMeasurement(0, 0);
  EXPECT_FLOAT_EQ(0.1, data);
}

// The battery publisher's warning shows these messages.
std::string GetADCMeasurementError(const husarion_ugv_battery::ADCDataReader & reader)
{
  try {
    reader.GetADCMeasurement(0, 0);
  } catch (const std::runtime_error & e) {
    return e.what();
  }
  return "";
}

TEST_F(TestADCDataReader, MissingFileThrowsOpenError)
{
  std::filesystem::remove(data_file_path_);

  const auto error = GetADCMeasurementError(*data_reader_);
  EXPECT_NE(std::string::npos, error.find("Failed to open file")) << error;
}

TEST_F(TestADCDataReader, UnreadableValueThrowsReadError)
{
  std::ofstream(data_file_path_) << "not a number";

  const auto error = GetADCMeasurementError(*data_reader_);
  EXPECT_NE(std::string::npos, error.find("Failed to read from file")) << error;
}

// Fails `failures` reads with `error`, starting at call `first_failing_call`, and reads the file
// otherwise.
class FailingRead
{
public:
  FailingRead(const int failures, const int error, const int first_failing_call = 1)
  : failures_(failures), error_(error), first_failing_call_(first_failing_call)
  {
  }

  husarion_ugv_battery::ADCDataReader::ReadFunction Function()
  {
    return [this](int fd, void * buf, std::size_t count) -> ssize_t {
      ++calls;
      if (calls >= first_failing_call_ && failures_ > 0) {
        --failures_;
        errno = error_;
        return -1;
      }
      return ::read(fd, buf, count);
    };
  }

  int calls = 0;

private:
  int failures_;
  const int error_;
  const int first_failing_call_;
};

TEST_F(TestADCDataReader, ReadRetriedOnceOnEIO)
{
  WriteNumberToFile<int>(1420, data_file_path_);
  FailingRead read(1, EIO);
  husarion_ugv_battery::ADCDataReader reader(testing::TempDir(), read.Function());

  EXPECT_FLOAT_EQ(1.42, reader.GetADCMeasurement(0, 0));
  // The scale read and the raw read, plus the one retry.
  EXPECT_EQ(3, read.calls);
}

TEST_F(TestADCDataReader, RawReadRetriedOnceOnEIO)
{
  WriteNumberToFile<int>(1420, data_file_path_);
  // GetADCMeasurement reads the scale first, so call 2 is the raw read.
  FailingRead read(1, EIO, 2);
  husarion_ugv_battery::ADCDataReader reader(testing::TempDir(), read.Function());

  EXPECT_FLOAT_EQ(1.42, reader.GetADCMeasurement(0, 0));
  EXPECT_EQ(3, read.calls);
}

TEST_F(TestADCDataReader, UnreadableValueAfterEIOReportsInvalidValue)
{
  std::ofstream(data_file_path_) << "not a number";
  FailingRead read(1, EIO);
  husarion_ugv_battery::ADCDataReader reader(testing::TempDir(), read.Function());

  // The error of the last attempt is reported, not the EIO of the first one.
  const auto error = GetADCMeasurementError(reader);
  EXPECT_NE(std::string::npos, error.find("invalid value")) << error;
  EXPECT_EQ(std::string::npos, error.find(std::strerror(EIO))) << error;
}

TEST_F(TestADCDataReader, ReadNotRetriedOnOtherErrors)
{
  WriteNumberToFile<int>(1420, data_file_path_);
  FailingRead read(1, ETIMEDOUT);
  husarion_ugv_battery::ADCDataReader reader(testing::TempDir(), read.Function());

  const auto error = GetADCMeasurementError(reader);
  EXPECT_NE(std::string::npos, error.find("Failed to read from file")) << error;
  EXPECT_NE(std::string::npos, error.find(std::strerror(ETIMEDOUT))) << error;
  EXPECT_EQ(1, read.calls);
}

TEST_F(TestADCDataReader, ReadRetriedOnlyOnce)
{
  WriteNumberToFile<int>(1420, data_file_path_);
  FailingRead read(2, EIO);
  husarion_ugv_battery::ADCDataReader reader(testing::TempDir(), read.Function());

  const auto error = GetADCMeasurementError(reader);
  EXPECT_NE(std::string::npos, error.find("Failed to read from file")) << error;
  EXPECT_EQ(2, read.calls);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
