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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_UVP_E_STOP_GUARD_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_UVP_E_STOP_GUARD_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>

namespace husarion_ugv_hardware_interfaces
{

// The parts of husarion_ugv_msgs/BatteryProtection the e-stop guard acts on.
struct UvpProtectionInput
{
  bool received = false;
  // BatteryProtection STATE_* and MODE_* values.
  std::uint8_t state = 0;
  std::uint8_t mode = 0;
  bool reactions_enabled = false;
  float drive_window_s = 60.0f;
  std::uint32_t trip_id = 0;
  std::int64_t trip_stamp_ns = 0;

  // Copies of the message constants, tied to them by static_asserts in ugv_system.cpp, so the
  // guard and its unit test need neither rclcpp nor the message package.
  static constexpr std::uint8_t kStateUvp = 3;
  static constexpr std::uint8_t kModeStop = 0;
};

/**
 * @brief Holds the e-stop on a robot whose battery undervoltage protection tripped in STOP mode.
 *
 * Triggers the e-stop once per trip. While the trip lasts, any release of the e-stop - a reset
 * service call or whatever else released it - opens a drive window, and the e-stop is triggered
 * again when the window ends. Releasing it again opens a new window. The guard only ever triggers
 * the e-stop: releasing it stays with the reset service and its checks.
 *
 * Run from the control loop, which reads the e-stop every cycle and so sees every release. All
 * state the diagnostics read is atomic.
 */
class UvpEStopGuard
{
public:
  // The battery driver's bounds for uvp.drive_window_s. The topic is not trusted: a window that is
  // not a finite positive number falls back to the default, and a longer one is capped.
  static constexpr double kDefaultDriveWindowS = 60.0;
  static constexpr double kMaxDriveWindowS = 300.0;

  static std::int64_t DriveWindowNs(const float drive_window_s);

  /**
   * @param trigger_e_stop Triggers the e-stop, returns false when that failed so the guard tries
   * again in the next cycle.
   */
  void Update(
    const UvpProtectionInput & input, const bool e_stop_latched, const std::int64_t now_ns,
    const std::function<bool()> & trigger_e_stop);

  bool Active() const { return active_.load(); }
  // Steady time the drive window ends, 0 when no window is open.
  std::int64_t GetWindowEndNs() const { return window_end_ns_.load(); }
  std::uint32_t GetHandledTripId() const { return handled_trip_id_.load(); }
  std::uint64_t GetTriggerCount() const { return trigger_count_.load(); }

private:
  bool Trigger(const std::function<bool()> & trigger_e_stop);

  // A trip is the trip id and its stamp: a battery driver that restarted without its latch counts
  // trips from 1 again, and the stamp tells such a new trip from an old one.
  std::optional<std::pair<std::uint32_t, std::int64_t>> handled_trip_;
  bool e_stop_latched_before_ = true;
  std::optional<std::int64_t> window_end_ns_value_;
  // A window-end trigger failed. The e-stop may read as latched while the safety board line is
  // still released, so the read-back is not trusted until a trigger goes through.
  bool window_end_trigger_pending_ = false;

  std::atomic_bool active_{false};
  std::atomic<std::int64_t> window_end_ns_{0};
  std::atomic<std::uint32_t> handled_trip_id_{0};
  std::atomic<std::uint64_t> trigger_count_{0};
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_UVP_E_STOP_GUARD_HPP_
