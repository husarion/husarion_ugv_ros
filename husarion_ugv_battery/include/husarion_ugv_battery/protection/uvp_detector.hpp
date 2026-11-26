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

#ifndef HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_PROTECTION_UVP_DETECTOR_HPP_
#define HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_PROTECTION_UVP_DETECTOR_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace husarion_ugv_battery
{

// Values match husarion_ugv_msgs/BatteryProtection STATE_*. Ordered by severity.
enum class UvpState : std::uint8_t {
  UNKNOWN = 0,
  OK = 1,
  CHARGE_ASAP = 2,
  UVP = 3,
};

std::string UvpStateToString(const UvpState state);

struct UvpDetectorConfig
{
  float uvp_threshold = 31.0f;
  float charge_asap_offset = 0.5f;
  unsigned consecutive_samples = 2;
  float clear_hysteresis = 0.5f;
  double charging_debounce_s = 5.0;
  // No valid read for longer than this makes the protection blind.
  double blind_timeout_s = 2.0;
};

// What survives a battery driver restart. The trip time is on CLOCK_BOOTTIME, so the time since
// the trip stays right across restarts and a reboot is recognisable.
struct UvpLatch
{
  std::string boot_id;
  std::uint32_t trip_id = 0;
  std::int64_t trip_boottime_ns = 0;
  std::int64_t trip_stamp_ns = 0;
  // Start of the time without charging: the trip, or the end of a charge after it.
  std::int64_t countdown_boottime_ns = 0;
  std::vector<bool> battery_uvp;
  std::vector<float> trip_voltage;
  std::vector<float> trip_current;
};

std::string SerializeUvpLatch(const UvpLatch & latch);
// Throws std::runtime_error on a malformed latch.
UvpLatch ParseUvpLatch(const std::string & text);

/**
 * @brief Battery undervoltage detection on the raw voltage, per battery.
 *
 * Each battery trips CHARGE_ASAP and UVP after a number of consecutive valid reads below the
 * threshold, with an independent counter per threshold. Both states latch. CHARGE_ASAP clears when
 * the battery charges, UVP when it charges and the voltage is back above the threshold plus a
 * hysteresis. Charging counts only after it held for the debounce time. The robot state is the
 * most severe battery state. No ROS dependency, every clock is injected.
 */
class UvpDetector
{
public:
  struct Clocks
  {
    // Debounce and blind detection.
    std::function<std::int64_t()> steady_ns;
    // Time since the trip, survives driver restarts.
    std::function<std::int64_t()> boottime_ns;
    // Trip stamp for the message.
    std::function<std::int64_t()> ros_ns;
  };

  UvpDetector(
    const UvpDetectorConfig & config, const std::size_t battery_count, const Clocks & clocks);

  /**
   * @brief Feeds one voltage read. A read is valid when the voltage is within the plausible range
   * and the battery's thermistor reads a temperature - an absent or disconnected pack reads below
   * the floor or with an open thermistor. Invalid reads are counted and leave the counters as they
   * are, unless no valid read came for longer than the blind timeout: the next valid read then
   * counts from scratch, so two low reads far apart are not consecutive.
   */
  void FeedVoltage(const std::size_t battery, const float voltage, const bool temperature_valid);

  // The latest discharge current, recorded with a trip in the same read.
  void FeedCurrent(const std::size_t battery, const float current);

  // The battery's charging status in this tick, debounced here.
  void FeedCharging(const std::size_t battery, const bool charging);

  UvpState GetState() const;
  UvpState GetBatteryState(const std::size_t battery) const { return batteries_.at(battery).state; }
  std::size_t GetBatteryCount() const { return batteries_.size(); }

  float GetUvpThreshold() const { return config_.uvp_threshold; }
  float GetChargeAsapThreshold() const
  {
    return config_.uvp_threshold + config_.charge_asap_offset;
  }

  // True when every battery has charged for the debounce time.
  bool IsCharging() const;
  // True when every battery reported charging in its last tick, not debounced.
  bool IsChargerConnected() const;
  // True when a battery had no valid read for longer than the blind timeout.
  bool IsBlind() const;
  std::uint64_t GetInvalidReads(const std::size_t battery) const
  {
    return batteries_.at(battery).invalid_reads;
  }

  std::uint32_t GetTripId() const { return trip_id_; }
  std::int64_t GetTripStampNs() const { return trip_stamp_ns_; }
  // NaN when the robot never tripped.
  double GetSecondsSinceTrip() const;
  // Time without charging while UVP is latched: since the trip, or since the last debounced charge
  // ended. NaN when UVP is not latched or while charging.
  double GetSecondsWithoutCharging() const;
  float GetTripVoltage(const std::size_t battery) const
  {
    return batteries_.at(battery).trip_voltage;
  }
  float GetTripCurrent(const std::size_t battery) const
  {
    return batteries_.at(battery).trip_current;
  }

  // Increments on every change the published message shows.
  std::uint64_t GetRevision() const { return revision_; }
  // Increments on every UVP trip and clear of a battery and at the end of a charge while UVP is
  // latched, the moments the latch is written.
  std::uint64_t GetLatchRevision() const { return latch_revision_; }

  UvpLatch GetLatch(const std::string & boot_id) const;

  /**
   * @brief Restores a latch written by an earlier run of the driver. Refused, with the reason in
   * why, when it comes from another boot or another battery configuration, or when the boot id is
   * unknown.
   */
  bool RestoreLatch(const UvpLatch & latch, const std::string & boot_id, std::string & why);

private:
  struct BatteryDetector
  {
    UvpState state = UvpState::UNKNOWN;
    unsigned below_uvp_count = 0;
    unsigned below_asap_count = 0;
    float last_valid_voltage = std::numeric_limits<float>::quiet_NaN();
    float last_current = std::numeric_limits<float>::quiet_NaN();
    std::int64_t last_valid_ns = 0;
    std::uint64_t invalid_reads = 0;
    std::optional<std::int64_t> charging_since_ns;
    bool charging = false;
    bool charger_connected = false;
    bool trip_current_pending = false;
    float trip_voltage = std::numeric_limits<float>::quiet_NaN();
    float trip_current = std::numeric_limits<float>::quiet_NaN();
  };

  void Evaluate(const std::size_t battery);
  void SetBatteryState(const std::size_t battery, const UvpState state);
  bool IsPlausible(const float voltage) const;

  static constexpr float kMinPlausibleVoltage = 20.0f;
  static constexpr float kMaxPlausibleVoltage = 50.0f;

  const UvpDetectorConfig config_;
  const Clocks clocks_;
  const std::int64_t start_ns_;
  std::vector<BatteryDetector> batteries_;

  std::uint32_t trip_id_ = 0;
  std::int64_t trip_boottime_ns_ = 0;
  std::int64_t trip_stamp_ns_ = 0;
  std::int64_t countdown_boottime_ns_ = 0;
  std::uint64_t revision_ = 0;
  std::uint64_t latch_revision_ = 0;
};

}  // namespace husarion_ugv_battery

#endif  // HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_PROTECTION_UVP_DETECTOR_HPP_
