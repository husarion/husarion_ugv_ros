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
  // Since the trip or the end of the last charge after it, as the battery driver counts it. NaN
  // when unknown.
  double seconds_without_charging = 0.0;
  // Debounced by the battery driver.
  bool charging = false;
  // Not debounced.
  bool charger_connected = false;
  // How long ago the message arrived.
  double sample_age_s = 0.0;

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
 * once the robot went the stop timeout T without charging. T counts from the trip, or from the end
 * of the last charge after it, as the battery driver reports it, so a restart of the safety manager
 * neither extends nor shortens it. Charging cancels T.
 * In SHUTDOWN mode it asks for the shutdown once the robot has not charged for
 * a confirm time (10 s) since the trip, counting a charger the battery driver has not
 * debounced yet: a robot that a charger powers on would otherwise trip and shut down before the
 * charging counts.
 * Data older than the stale timeout keeps the trip, but no longer counts as charging: a battery
 * driver that stopped publishing cannot hold the shutdown. No ROS dependency, the time is passed
 * in.
 */
class UvpReaction
{
public:
  // Longer than the battery driver needs to learn the charger state after a start.
  static constexpr std::int64_t kDefaultShutdownNoChargeConfirmNs = 10'000'000'000;
  static constexpr double kDefaultStopTimeoutMin = 5.0;
  static constexpr double kMaxStopTimeoutMin = 60.0;
  static constexpr float kDefaultShutdownHostsTimeoutS = 10.0f;
  static constexpr float kMaxShutdownHostsTimeoutS = 30.0f;
  // A charger that reads connected but never charges holds a SHUTDOWN-mode shutdown for at most
  // this long: twice the longest uvp.charging_debounce_s, so a real charger always gets to charge.
  static constexpr std::int64_t kMaxChargerHoldNs = 60'000'000'000;
  // The battery driver publishes at least once a second.
  static constexpr double kDefaultStaleTimeoutS = 3.0;

  explicit UvpReaction(
    const std::int64_t shutdown_no_charge_confirm_ns = kDefaultShutdownNoChargeConfirmNs,
    const double stale_timeout_s = kDefaultStaleTimeoutS)
  : shutdown_no_charge_confirm_ns_(shutdown_no_charge_confirm_ns), stale_timeout_s_(stale_timeout_s)
  {
  }

  // The topic is not trusted: a value that is not a finite positive number means the default, a
  // larger one is capped at the battery driver's maximum. Below the driver's minimum is allowed,
  // which the tests use for a T of seconds.
  static double StopTimeoutMin(const float stop_timeout_min);
  static float ShutdownHostsTimeoutS(const float shutdown_hosts_timeout_s);

  bool Stale(const UvpReactionInput & input) const
  {
    return input.received && input.sample_age_s > stale_timeout_s_;
  }

  UvpReactionActions Update(const UvpReactionInput & input, const std::int64_t now_ns);

  // True while a trip with reactions holds: every shutdown then takes the UVP sequence.
  bool Latched() const { return latched_; }
  // Seconds until the shutdown, empty when none is due.
  std::optional<double> SecondsToShutdown(const std::int64_t now_ns) const;
  bool ShutdownRequested() const { return shutdown_requested_; }

private:
  const std::int64_t shutdown_no_charge_confirm_ns_;
  const double stale_timeout_s_;
  std::optional<std::pair<std::uint32_t, std::int64_t>> trip_;
  std::optional<std::int64_t> deadline_ns_;
  std::optional<std::int64_t> no_charge_since_ns_;
  // Since when the charger reads connected in SHUTDOWN mode without the charging confirmed.
  std::optional<std::int64_t> charger_only_since_ns_;
  bool latched_ = false;
  bool shutdown_requested_ = false;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_UVP_REACTION_HPP_
