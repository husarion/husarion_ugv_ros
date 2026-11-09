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

#ifndef HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_ADC_DATA_READER_HPP_
#define HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_ADC_DATA_READER_HPP_

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace husarion_ugv_battery
{

class ADCDataReader
{
public:
  using ReadFunction = std::function<ssize_t(int, void *, std::size_t)>;

  // read_function exists so that tests can inject read errors.
  ADCDataReader(const std::string & device_path, ReadFunction read_function = ::read)
  : device_path_(device_path), read_function_(std::move(read_function))
  {
    if (!std::filesystem::exists(device_path_)) {
      throw std::runtime_error(
        "Device does not exists under given path:" + std::string(device_path_));
    }
  }

  float GetADCMeasurement(const int channel, const int offset) const
  {
    const auto LSB = ReadChannel<float>(channel, "scale") / 1000.0f;
    const auto raw_value = ReadChannel<int>(channel, "raw");
    return (raw_value - offset) * LSB;
  }

private:
  // Only EIO is retried, once. Any other error, a timeout included, fails at once. The error text
  // goes into the exception, so the log shows which errors really occur.
  static constexpr int kReadAttempts = 2;

  template <typename T>
  T ReadChannel(const int channel, const std::string & data_type) const
  {
    if (data_type != "raw" && data_type != "scale" && data_type != "sampling_frequency") {
      throw std::logic_error("Invalid data type: " + data_type);
    }

    const auto data_file = "in_voltage" + std::to_string(channel) + "_" + data_type;
    const auto file_path = device_path_ / data_file;

    return ReadFile<T>(file_path);
  }

  template <typename T>
  T ReadFile(const std::filesystem::path & file_path) const
  {
    std::array<char, 64> buffer{};
    std::string error;
    for (int attempt = 1;; ++attempt) {
      const int fd = ::open(file_path.c_str(), O_RDONLY | O_CLOEXEC);
      if (fd < 0) {
        throw std::runtime_error(
          "Failed to open file: " + std::string(file_path) + ": " + std::strerror(errno));
      }
      const ssize_t length = read_function_(fd, buffer.data(), buffer.size() - 1);
      const int read_errno = errno;
      ::close(fd);

      if (length >= 0) {
        std::istringstream stream(std::string(buffer.data(), static_cast<std::size_t>(length)));
        T data;
        stream >> data;
        if (stream) {
          return data;
        }
        error = "invalid value";
        break;
      }
      error = std::strerror(read_errno);
      if (read_errno != EIO || attempt >= kReadAttempts) {
        break;
      }
    }
    throw std::runtime_error("Failed to read from file: " + std::string(file_path) + ": " + error);
  }

  const std::filesystem::path device_path_;
  const ReadFunction read_function_;
};

}  // namespace husarion_ugv_battery

#endif  // HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_ADC_DATA_READER_HPP_
