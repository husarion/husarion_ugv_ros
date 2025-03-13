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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_UTILS_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_UTILS_HPP_

#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>

namespace husarion_ugv_hardware_interfaces
{

/**
 * @brief Get byte byte_no from data (byte_no has to be in [0;sizeof(T)] range)
 * @exception std::runtime_error if byte_no is out of range
 */
template <typename T>
std::uint8_t GetByte(const T data, const unsigned byte_no)
{
  if (byte_no >= sizeof(T)) {
    throw std::runtime_error(
      "byte_no out of range, allowed values: [0," + std::to_string(sizeof(T)) + "].");
  }
  return (data >> (byte_no * 8)) & 0xFF;
}

/**
 * @brief Attempts to run operation for max_attempts number of times.
 * operation can throw std::runtime_error, which is caught, and on_error function
 * is executed (for example deinitialization or some other cleanup in case of
 * failure)
 * @return true if the operation was successfully executed, false if it wasn't successfully executed
 * and the number of attempts exceeded the maximum allowed or on_error function threw
 * std::runtime_error
 */
bool OperationWithAttempts(
  const std::function<void()> operation, const unsigned max_attempts,
  const std::function<void()> on_error = []() {});

/**
 * @brief Checks whether joint name contains valid sequence, it can be used in three ways:
 * 1. Prefix sequence_jointname or namespace/sequence_jointname
 * 2. Infix jointname1_sequence_jointname2
 * 3. Postfix jointname_sequence
 * @return true if sequence is present in name and is valid
 */
bool CheckIfJointNameContainValidSequence(const std::string & name, const std::string & sequence);

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_UTILS_HPP_
