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

#ifndef HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_UVP_REACTION_HPP_
#define HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_UVP_REACTION_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace husarion_ugv_manager
{

// The parts of husarion_ugv_msgs/BatteryProtection the safety manager acts on.
struct UvpReactionInput
{
  bool received = false;
  // BatteryProtection STATE_* and MODE_* values.
  std::uint8_t state = 0;
  std::uint8_t mode = 0;
  bool reactions_enabled = false;
  float stop_timeout_min = 5.0f;
  bool aux_off_at_trip = false;
  float shutdown_hosts_timeout_s = 10.0f;
  std::uint32_t trip_id = 0;
  std::int64_t trip_stamp_ns = 0;
  // NaN when unknown.
  float seconds_since_trip = 0.0f;
  bool charging = false;

  static constexpr std::uint8_t kStateUvp = 3;
  static constexpr std::uint8_t kModeStop = 0;
  static constexpr std::uint8_t kModeShutdown = 1;
};

struct UvpReactionActions
{
  bool aux_off = false;
  bool shutdown = false;
  std::string reason;
};

/**
 * @brief The safety manager's part of the battery undervoltage protection.
 *
 * In STOP mode it turns the AUX power off at the trip when asked to, and asks for the UVP shutdown
 * once the robot went the stop timeout T without charging. T counts from the trip, as the battery
 * driver reports it, so a restart of the safety manager does not start it again. Charging cancels
 * T. When the charging ends with the protection still latched, a full T starts from that moment.
 * In SHUTDOWN mode it asks for the shutdown at the trip. No ROS dependency, the time is passed in.
 */
class UvpReaction
{
public:
  UvpReactionActions Update(const UvpReactionInput & input, const std::int64_t now_ns);

  // True while a trip with reactions holds: every shutdown then takes the UVP sequence.
  bool Latched() const { return latched_; }
  // Seconds until the shutdown, empty when none is due.
  std::optional<double> SecondsToShutdown(const std::int64_t now_ns) const;
  bool ShutdownRequested() const { return shutdown_requested_; }

private:
  std::optional<std::pair<std::uint32_t, std::int64_t>> trip_;
  std::optional<std::int64_t> deadline_ns_;
  bool latched_ = false;
  bool shutdown_requested_ = false;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_UVP_REACTION_HPP_
