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

#ifndef HUSARION_UGV_UTILS_TEST_UTILS_HPP_
#define HUSARION_UGV_UTILS_TEST_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace husarion_ugv_utils::test_utils
{

/**
 * @brief Check if all elements of vector are NaN
 *
 * @param vector Vector that will be checked
 *
 * @return True if all elements of the vector are NaN, false otherwise
 * @exception std::runtime_error if method has invalid typename,
 * valid options are: 'float', 'double', 'long double'
 */
template <typename T>
bool CheckNaNVector(const std::vector<T> & vector)
{
  if (!std::numeric_limits<T>::has_quiet_NaN) {
    throw std::runtime_error(
      "Invalid method typename. Valid are: 'float', 'double', 'long double'.");
  }
  return std::all_of(vector.begin(), vector.end(), [](const T value) { return std::isnan(value); });
}

/**
 * @brief Check if a method throws an exception of a given type and the error message contains the
 * provided message
 *
 * @param func The method that will be tested
 * @param error_msg The error message that has to be contained within the thrown message
 *
 * @return True if method throws an exception of a given type and the error message contains the
 * provided message, false otherwise
 */
template <typename ExceptionType, typename Func>
bool IsMessageThrown(const Func & func, const std::string & error_msg)
{
  try {
    func();
  } catch (const ExceptionType & e) {
    if (std::string(e.what()).find(error_msg) != std::string::npos) {
      return true;
    }
  } catch (...) {
  }

  return false;
}

}  // namespace husarion_ugv_utils::test_utils

#endif  // HUSARION_UGV_UTILS_TEST_UTILS_HPP_
