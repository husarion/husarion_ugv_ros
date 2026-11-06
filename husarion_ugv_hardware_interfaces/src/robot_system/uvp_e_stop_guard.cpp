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

#include "husarion_ugv_hardware_interfaces/robot_system/uvp_e_stop_guard.hpp"

#include <cstdint>
#include <functional>
#include <utility>

namespace husarion_ugv_hardware_interfaces
{

void UvpEStopGuard::Update(
  const UvpProtectionInput & input, const bool e_stop_latched, const std::int64_t now_ns,
  const std::function<bool()> & trigger_e_stop)
{
  const bool uvp = input.received && input.state == UvpProtectionInput::kStateUvp;
  const bool active = uvp && input.reactions_enabled && input.mode == UvpProtectionInput::kModeStop;
  active_.store(active);

  if (!active) {
    window_end_ns_value_.reset();
    window_end_ns_.store(0);
    e_stop_latched_before_ = e_stop_latched;
    // A cleared protection makes the next trip a new one, whatever its id.
    if (input.received && !uvp) {
      handled_trip_.reset();
    }
    return;
  }

  // Acting on the state, not on its edge: a driver that starts while the battery driver already
  // reports a latched trip stops the robot all the same.
  const auto trip = std::make_pair(input.trip_id, input.trip_stamp_ns);
  if (handled_trip_ != trip) {
    window_end_ns_value_.reset();
    window_end_ns_.store(0);
    if (Trigger(trigger_e_stop)) {
      handled_trip_ = trip;
      handled_trip_id_.store(input.trip_id);
      e_stop_latched_before_ = true;
    }
    return;
  }

  // Whatever released the e-stop opens the window. A release inside an open window keeps its end,
  // so latching and releasing again cannot stretch it.
  if (e_stop_latched_before_ && !e_stop_latched && !window_end_ns_value_.has_value()) {
    window_end_ns_value_ = now_ns + static_cast<std::int64_t>(input.drive_window_s * 1e9);
    window_end_ns_.store(window_end_ns_value_.value());
  }
  e_stop_latched_before_ = e_stop_latched;

  if (window_end_ns_value_.has_value() && now_ns >= window_end_ns_value_.value()) {
    if (e_stop_latched) {
      // Latched by something else before the window ended: nothing left to do.
      window_end_ns_value_.reset();
      window_end_ns_.store(0);
    } else if (Trigger(trigger_e_stop)) {
      window_end_ns_value_.reset();
      window_end_ns_.store(0);
      e_stop_latched_before_ = true;
    }
  }
}

bool UvpEStopGuard::Trigger(const std::function<bool()> & trigger_e_stop)
{
  if (!trigger_e_stop()) {
    return false;
  }
  trigger_count_++;
  return true;
}

}  // namespace husarion_ugv_hardware_interfaces
