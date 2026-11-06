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

#include "husarion_ugv_manager/uvp_reaction.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace husarion_ugv_manager
{

UvpReactionActions UvpReaction::Update(const UvpReactionInput & input, const std::int64_t now_ns)
{
  UvpReactionActions actions;

  latched_ = input.received && input.state == UvpReactionInput::kStateUvp &&
             input.reactions_enabled;
  if (!latched_) {
    trip_.reset();
    deadline_ns_.reset();
    return actions;
  }

  // Requested once: the shutdown sequence ends this node.
  if (shutdown_requested_) {
    return actions;
  }

  const auto stop_timeout_ns =
    static_cast<std::int64_t>(static_cast<double>(input.stop_timeout_min) * 60.0 * 1e9);

  const auto trip = std::make_pair(input.trip_id, input.trip_stamp_ns);
  if (trip_ != trip) {
    trip_ = trip;
    deadline_ns_.reset();

    if (input.mode == UvpReactionInput::kModeShutdown) {
      shutdown_requested_ = true;
      actions.shutdown = true;
      actions.reason = "Battery undervoltage protection (SHUTDOWN mode)";
      return actions;
    }

    actions.aux_off = input.aux_off_at_trip;
    if (!input.charging) {
      const double since_trip = std::isnan(input.seconds_since_trip)
                                  ? 0.0
                                  : std::max(0.0, static_cast<double>(input.seconds_since_trip));
      deadline_ns_ = now_ns + stop_timeout_ns - static_cast<std::int64_t>(since_trip * 1e9);
    }
  } else if (input.charging) {
    deadline_ns_.reset();
  } else if (!deadline_ns_.has_value()) {
    // The charging ended while the protection is still latched.
    deadline_ns_ = now_ns + stop_timeout_ns;
  }

  if (deadline_ns_.has_value() && now_ns >= deadline_ns_.value()) {
    shutdown_requested_ = true;
    actions.shutdown = true;
    actions.reason = "Battery undervoltage protection (no charging within " +
                     std::to_string(static_cast<int>(input.stop_timeout_min)) + " min)";
  }

  return actions;
}

std::optional<double> UvpReaction::SecondsToShutdown(const std::int64_t now_ns) const
{
  if (!deadline_ns_.has_value()) {
    return std::nullopt;
  }
  return std::max(0.0, static_cast<double>(deadline_ns_.value() - now_ns) / 1e9);
}

}  // namespace husarion_ugv_manager
