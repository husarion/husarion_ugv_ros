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

#ifndef HUSARION_UGV_UTILS_COMMON_UTILITIES_HPP_
#define HUSARION_UGV_UTILS_COMMON_UTILITIES_HPP_

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace husarion_ugv_utils::common_utilities
{

/**
 * @brief Sets the precision of a floating point value.
 *
 * @tparam T The type of the value - must be floating point type.
 * @param value The value to set the precision of.
 * @param precision The precision to set.
 * @return The value with the set precision.
 */
template <typename T>
T SetPrecision(T value, unsigned int precision)
{
  static_assert(
    std::is_floating_point<T>::value,
    "SetPrecision method can only be used with floating point types.");

  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;

  return std::stof(out.str());
}

/**
 * @brief Counts the percentage of a value in relation to a total. Result is returned as two digit
 * precision float.
 *
 * @param value The share value.
 * @param total The total value.
 * @return The percentage value.
 */
template <typename T>
float CountPercentage(const T & value, const T & total)
{
  if (total == 0) {
    throw std::invalid_argument("Total must not be zero.");
  }

  auto percentage = static_cast<float>(value) / static_cast<float>(total) * 100.0;
  return SetPrecision(percentage, 2);
}

/**
 * @brief Prefixes the keys of a given map with a specified prefix.
 *
 * @tparam T The value type of the map.
 * @param map The input map. Key of the map must be std::string type.
 * @param prefix The prefix to be added to each key.
 * @return A new map with the keys prefixed.
 */
template <typename T>
std::map<std::string, T> PrefixMapKeys(
  const std::map<std::string, T> & map, const std::string & prefix)
{
  std::map<std::string, T> prefixed_map;
  for (const auto & [key, value] : map) {
    prefixed_map[prefix + key] = value;
  }
  return prefixed_map;
}

/**
 * @brief Opens a file with the specified mode.
 *
 * This function opens a file with the specified mode and returns a std::fstream object.
 * If the file fails to open, it throws a std::runtime_error.
 *
 * @param file_path The path to the file to be opened.
 * @param mode The mode in which the file should be opened (e.g., std::ios_base::in |
 * std::ios_base::out).
 * @return std::fstream The opened file stream.
 *
 * @throws std::runtime_error if the file fails to open.
 */
inline std::fstream OpenFile(const std::string & file_path, const std::ios_base::openmode & mode)
{
  std::fstream file(file_path, mode);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + file_path);
  }
  return file;
}

/**
 * @brief Checks if required version is satisfied.
 *
 * @param version Version to be verified.
 * @param required_version The required version.
 * @return true if the required version is satisfied, false otherwise.
 */
inline bool MeetsVersionRequirement(const float version, const float required_version)
{
  if (std::isnan(version) || std::isnan(required_version)) {
    return false;
  }

  return version >= required_version - std::numeric_limits<float>::epsilon();
}

}  // namespace husarion_ugv_utils::common_utilities

#endif  // HUSARION_UGV_UTILS_COMMON_UTILITIES_HPP_
