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

#include "husarion_ugv_hardware_interfaces/utils.hpp"

#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace husarion_ugv_hardware_interfaces
{

bool OperationWithAttempts(
  const std::function<void()> operation, const unsigned max_attempts,
  const std::function<void()> on_error)
{
  for (unsigned i = 0; i < max_attempts; ++i) {
    try {
      operation();
      return true;
    } catch (const std::runtime_error & e) {
      std::cerr << "An exception occurred while handling operation() function, attempt " << i + 1
                << " of " << max_attempts << ": " << e.what() << std::endl;
      try {
        on_error();
      } catch (const std::runtime_error & on_error_e) {
        std::cerr << "An exception occurred while handling on_error() function: "
                  << on_error_e.what() << std::endl;
        return false;
      }
    }
  }
  return false;
}

bool CheckIfJointNameContainValidSequence(const std::string & name, const std::string & sequence)
{
  const std::size_t pos = name.find(sequence);
  if (pos == std::string::npos) {
    return false;
  }

  if (pos >= 1) {
    const std::size_t id_before_sequence = pos - 1;
    if (name[id_before_sequence] != '_' && name[id_before_sequence] != '/') {
      return false;
    }
  }

  const std::size_t id_after_sequence = pos + sequence.length();
  if (id_after_sequence < name.length() && name[id_after_sequence] != '_') {
    return false;
  }

  return true;
}

// Strips leading zeros and returns the remaining digits, or an empty string if text is not a
// whole number. "0" and "000" give "0".
static std::string WholeNumberDigits(const std::string & text)
{
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    return "";
  }
  const auto first_digit = text.find_first_not_of('0');
  return first_digit == std::string::npos ? "0" : text.substr(first_digit);
}

std::optional<unsigned> ParseRTPriority(const std::string & text)
{
  const std::string digits = WholeNumberDigits(text);
  // At most three digits, so stoul can neither overflow nor accept a sign or spaces.
  if (digits.empty() || digits.size() > 3) {
    return std::nullopt;
  }
  const unsigned priority = std::stoul(digits);
  if (priority > kMaxRTPriority) {
    return std::nullopt;
  }
  return priority;
}

std::optional<int> ParseCPU(const std::string & text, const int cpu_count)
{
  if (text == "-1") {
    return -1;
  }
  const std::string digits = WholeNumberDigits(text);
  if (digits.empty() || digits.size() > 4) {
    return std::nullopt;
  }
  const int cpu = std::stoi(digits);
  if (cpu >= cpu_count) {
    return std::nullopt;
  }
  return cpu;
}

std::string MakeDCFUploadFilesAbsolute(
  const std::string & dcf, const std::filesystem::path & dcf_dir)
{
  const std::string upload_file_key = "UploadFile=";
  std::istringstream source(dcf);
  std::ostringstream result;
  std::string line;
  while (std::getline(source, line)) {
    // A CR left on the path would make every slave boot fail to open the file.
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.rfind(upload_file_key, 0) == 0) {
      const std::filesystem::path upload_file = line.substr(upload_file_key.size());
      if (upload_file.is_relative()) {
        line = upload_file_key + (dcf_dir / upload_file).string();
      }
    }
    result << line << '\n';
  }
  return result.str();
}

}  // namespace husarion_ugv_hardware_interfaces
