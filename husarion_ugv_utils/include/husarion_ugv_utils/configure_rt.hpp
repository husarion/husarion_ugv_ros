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

#ifndef HUSARION_UGV_UTILS_CONFIGURE_RT_HPP_
#define HUSARION_UGV_UTILS_CONFIGURE_RT_HPP_

#include <stdexcept>

#include "realtime_tools/realtime_helpers.hpp"

namespace husarion_ugv_utils
{
/**
 * @brief Configures thread that calls this function to FIFO scheduler with RT priority
 *
 * @param priority RT priority of thread (value in [0, 99] range)
 *
 * @exception std::runtime_error if invalid priority is set, kernel isn't RT or configuration fails.
 */
inline void ConfigureRT(const unsigned priority)
{
  if (priority > 99) {
    throw std::runtime_error(
      "Invalid priority value. Please set a value between 0 and 99 for RT scheduling.");
  }

  if (!realtime_tools::has_realtime_kernel()) {
    throw std::runtime_error("Real-time kernel is not available.");
  }

  if (!realtime_tools::configure_sched_fifo(priority)) {
    throw std::runtime_error("Could not enable FIFO RT scheduling policy.");
  }
}

}  // namespace husarion_ugv_utils

#endif  // HUSARION_UGV_UTILS_CONFIGURE_RT_HPP_
