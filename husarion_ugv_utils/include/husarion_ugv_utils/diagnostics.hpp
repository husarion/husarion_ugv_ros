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

#ifndef HUSARION_UGV_UTILS_DIAGNOSTICS_HPP_
#define HUSARION_UGV_UTILS_DIAGNOSTICS_HPP_

#include <map>
#include <string>

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

namespace husarion_ugv_utils::diagnostics
{

/**
 * @brief Adds key-value pairs to a diagnostic status if the value is true.
 *
 * This function iterates over a map of key-value pairs and adds each pair to the diagnostic status
 * if the value is true. The key prefix and suffix can be optionally provided to modify the keys
 * before adding them to the status.
 *
 * @tparam ValueT The type of the values in the map, must be evaluable to boolean context.
 * @param status The diagnostic status wrapper to add the key-value pairs to.
 * @param kv_map The map of key-value pairs.
 * @param key_prefix The optional prefix to add to each key.
 * @param key_suffix The optional suffix to add to each key.
 */
template <typename ValueT>
void AddKeyValueIfTrue(
  diagnostic_updater::DiagnosticStatusWrapper & status,
  const std::map<std::string, ValueT> & kv_map, const std::string & key_prefix = "",
  const std::string & key_suffix = "")
{
  for (const auto & [key, value] : kv_map) {
    if (static_cast<bool>(value)) {
      status.add(key_prefix + key + key_suffix, value);
    }
  }
}

}  // namespace husarion_ugv_utils::diagnostics

#endif  // HUSARION_UGV_UTILS_DIAGNOSTICS_HPP_
