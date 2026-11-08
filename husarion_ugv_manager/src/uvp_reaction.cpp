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
#include <sstream>
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
    no_charge_since_ns_.reset();
    charger_only_since_ns_.reset();
    return actions;
  }

  // Requested once: the shutdown sequence ends this node.
  if (shutdown_requested_) {
    return actions;
  }

  const auto trip = std::make_pair(input.trip_id, input.trip_stamp_ns);
  const bool new_trip = trip_ != trip;
  if (new_trip) {
    trip_ = trip;
    deadline_ns_.reset();
    charger_only_since_ns_.reset();
  }

  const bool stale = Stale(input);
  const bool charging = input.charging && !stale;
  const bool charger_connected = input.charger_connected && !stale;
  // The message's counts are from when it was sent.
  const auto sample_age_ns = static_cast<std::int64_t>(std::max(0.0, input.sample_age_s) * 1e9);

  if (input.mode == UvpReactionInput::kModeShutdown) {
    bool hold = charging;
    if (charging) {
      charger_only_since_ns_.reset();
    } else if (charger_connected) {
      if (!charger_only_since_ns_.has_value()) {
        charger_only_since_ns_ = now_ns - sample_age_ns;
      }
      // A charger that never charges does not hold the shutdown for good, it counts from when it
      // was first seen.
      hold = now_ns - charger_only_since_ns_.value() < kMaxChargerHoldNs;
      if (!hold && !no_charge_since_ns_.has_value()) {
        no_charge_since_ns_ = charger_only_since_ns_;
      }
    }
    if (hold) {
      no_charge_since_ns_.reset();
      return actions;
    }
    if (!no_charge_since_ns_.has_value()) {
      no_charge_since_ns_ = now_ns - sample_age_ns;
    }
    if (now_ns - no_charge_since_ns_.value() >= shutdown_no_charge_confirm_ns_) {
      shutdown_requested_ = true;
      actions.shutdown = true;
      actions.reason = "Battery undervoltage protection (SHUTDOWN mode)";
    }
    return actions;
  }

  if (new_trip) {
    actions.aux_off = input.aux_off_at_trip;
  }

  const double stop_timeout_min = StopTimeoutMin(input.stop_timeout_min);
  const auto stop_timeout_ns = static_cast<std::int64_t>(stop_timeout_min * 60.0 * 1e9);
  if (charging) {
    deadline_ns_.reset();
  } else if (!deadline_ns_.has_value()) {
    // The battery driver's time is on CLOCK_BOOTTIME and the deadline on this node's steady clock.
    // Both keep counting across a restart of either node. Stale data that still said charging has
    // no such time, and T counts from that message. An unknown time is taken as none, which gives
    // the robot the full T to reach a charger rather than shutting it down at once.
    const double without_charging = std::isnan(input.seconds_without_charging)
                                      ? 0.0
                                      : std::max(0.0, input.seconds_without_charging);
    deadline_ns_ = now_ns + stop_timeout_ns - static_cast<std::int64_t>(without_charging * 1e9) -
                   sample_age_ns;
  }

  if (deadline_ns_.has_value() && now_ns >= deadline_ns_.value()) {
    shutdown_requested_ = true;
    actions.shutdown = true;
    std::ostringstream reason;
    reason << "Battery undervoltage protection (no charging within " << stop_timeout_min << " min)";
    actions.reason = reason.str();
  }

  return actions;
}

double UvpReaction::StopTimeoutMin(const float stop_timeout_min)
{
  if (!std::isfinite(stop_timeout_min) || stop_timeout_min <= 0.0f) {
    return kDefaultStopTimeoutMin;
  }
  return std::min(static_cast<double>(stop_timeout_min), kMaxStopTimeoutMin);
}

float UvpReaction::ShutdownHostsTimeoutS(const float shutdown_hosts_timeout_s)
{
  if (!std::isfinite(shutdown_hosts_timeout_s) || shutdown_hosts_timeout_s <= 0.0f) {
    return kDefaultShutdownHostsTimeoutS;
  }
  return std::min(shutdown_hosts_timeout_s, kMaxShutdownHostsTimeoutS);
}

std::optional<double> UvpReaction::SecondsToShutdown(const std::int64_t now_ns) const
{
  if (no_charge_since_ns_.has_value()) {
    return std::max(
      0.0,
      static_cast<double>(no_charge_since_ns_.value() + shutdown_no_charge_confirm_ns_ - now_ns) /
        1e9);
  }
  if (!deadline_ns_.has_value()) {
    return std::nullopt;
  }
  return std::max(0.0, static_cast<double>(deadline_ns_.value() - now_ns) / 1e9);
}

}  // namespace husarion_ugv_manager
