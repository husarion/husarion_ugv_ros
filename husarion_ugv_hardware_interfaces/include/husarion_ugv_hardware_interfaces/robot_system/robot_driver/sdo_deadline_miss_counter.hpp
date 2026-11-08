// Copyright 2026 Husarion sp. z o.o.
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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_SDO_DEADLINE_MISS_COUNTER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_SDO_DEADLINE_MISS_COUNTER_HPP_

#include <chrono>
#include <functional>
#include <mutex>
#include <utility>

namespace husarion_ugv_hardware_interfaces
{

/**
 * @brief Counts SDO writes whose confirmation missed its deadline. The window is fixed and starts
 * at the first miss: max_misses misses before it ends trip the counter, and a miss after it ends
 * starts a new count. A delivered confirmation clears the count.
 */
class SdoDeadlineMissCounter
{
public:
  using Clock = std::function<std::chrono::steady_clock::time_point()>;

  SdoDeadlineMissCounter(
    const unsigned max_misses, const std::chrono::steady_clock::duration window,
    Clock clock = []() { return std::chrono::steady_clock::now(); })
  : max_misses_(max_misses), window_(window), clock_(std::move(clock))
  {
  }

  /**
   * @brief Records a missed deadline.
   * @return true when this miss trips the counter
   */
  bool OnMiss()
  {
    std::lock_guard<std::mutex> lck(mtx_);
    const auto now = clock_();
    if (misses_ == 0 || now - first_miss_ > window_) {
      misses_ = 0;
      first_miss_ = now;
    }
    return ++misses_ >= max_misses_;
  }

  /**
   * @brief Records a confirmation that arrived in time.
   */
  void OnDelivered()
  {
    std::lock_guard<std::mutex> lck(mtx_);
    misses_ = 0;
  }

private:
  const unsigned max_misses_;
  const std::chrono::steady_clock::duration window_;
  const Clock clock_;

  std::mutex mtx_;
  unsigned misses_ = 0;
  std::chrono::steady_clock::time_point first_miss_;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_SDO_DEADLINE_MISS_COUNTER_HPP_
