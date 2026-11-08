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

#ifndef HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_PROTECTION_UVP_PARAMETERS_HPP_
#define HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_PROTECTION_UVP_PARAMETERS_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace husarion_ugv_battery
{

namespace uvp
{

// Ranges for the battery undervoltage protection parameters. A value outside its range is reported
// as an error in the diagnostics and replaced by the default - never a reason to stop the node,
// which would also take the battery dead shutdown down with it.
constexpr float kDefaultThreshold = 31.0f;
constexpr float kMinThreshold = 28.0f;
constexpr float kMaxThreshold = 36.0f;

constexpr float kDefaultChargeAsapOffset = 0.5f;
constexpr float kMinChargeAsapOffset = 0.2f;
constexpr float kMaxChargeAsapOffset = 3.0f;

constexpr int kDefaultConsecutiveSamples = 2;
constexpr int kMinConsecutiveSamples = 1;
constexpr int kMaxConsecutiveSamples = 10;

constexpr float kDefaultClearHysteresis = 0.5f;
constexpr float kMinClearHysteresis = 0.2f;
constexpr float kMaxClearHysteresis = 2.0f;

constexpr int kDefaultStopTimeoutMin = 5;
constexpr int kMinStopTimeoutMin = 1;
constexpr int kMaxStopTimeoutMin = 60;
// An existing external value of uvp.stop_timeout_min that turns every reaction off.
constexpr int kLegacyReactionsOffStopTimeoutMin = 998;

constexpr float kDefaultDriveWindow = 60.0f;
constexpr float kMinDriveWindow = 10.0f;
constexpr float kMaxDriveWindow = 300.0f;

constexpr float kDefaultShutdownHostsTimeout = 10.0f;
constexpr float kMinShutdownHostsTimeout = 1.0f;
constexpr float kMaxShutdownHostsTimeout = 30.0f;

constexpr float kDefaultChargingDebounce = 5.0f;
constexpr float kMinChargingDebounce = 1.0f;
constexpr float kMaxChargingDebounce = 30.0f;

constexpr char kDefaultHatEepromPath[] = "/proc/device-tree/hat/custom_0";
constexpr char kEepromThresholdKey[] = "robot.battery.uvp_threshold_v";

constexpr char kReactionsOffByConfig[] = "UVP reactions disabled by configuration";
constexpr char kReactionsOffNoCharging[] = "reactions off: charging cannot be detected";

enum class Mode : std::uint8_t { STOP = 0, SHUTDOWN = 1 };

}  // namespace uvp

enum class UvpNoteLevel : std::uint8_t { INFO, WARN, ERROR };

struct UvpNote
{
  UvpNoteLevel level;
  std::string text;
};

struct UvpParams
{
  bool enable = false;
  bool reactions_enabled = true;
  // Set and valid, else empty.
  std::optional<float> threshold;
  float charge_asap_offset = uvp::kDefaultChargeAsapOffset;
  int consecutive_samples = uvp::kDefaultConsecutiveSamples;
  float clear_hysteresis = uvp::kDefaultClearHysteresis;
  uvp::Mode mode = uvp::Mode::STOP;
  int stop_timeout_min = uvp::kDefaultStopTimeoutMin;
  bool aux_off_at_trip = false;
  float drive_window_s = uvp::kDefaultDriveWindow;
  float shutdown_hosts_timeout_s = uvp::kDefaultShutdownHostsTimeout;
  float charging_debounce_s = uvp::kDefaultChargingDebounce;
  std::string hat_eeprom_path = uvp::kDefaultHatEepromPath;
  std::string latch_path;

  // Rejected values, each replaced by its default.
  std::vector<UvpNote> notes;

  // uvp.reactions_enabled false, or its legacy alias uvp.stop_timeout_min 998.
  bool ReactionsOffByConfig() const
  {
    return !reactions_enabled || stop_timeout_min == uvp::kLegacyReactionsOffStopTimeoutMin;
  }
};

/**
 * @brief Declares the uvp.* parameters and reads them. Every parameter takes a bool, a number or a
 * string, so a value passed through a launch argument as text is read the same as a typed one. An
 * invalid value adds an error note and keeps the default.
 */
UvpParams DeclareUvpParameters(rclcpp::Node & node);

struct UvpThreshold
{
  float value = uvp::kDefaultThreshold;
  // "param", "eeprom" or "default".
  std::string source = "default";
  std::vector<UvpNote> notes;
};

/**
 * @brief Picks the UVP threshold: a valid parameter, else a valid key in the raw HAT EEPROM YAML,
 * else the built-in default.
 *
 * The kernel exposes the HAT EEPROM's first custom data atom at /proc/device-tree/hat/custom_0.
 * It holds a YAML document written when the robot is provisioned, and the threshold is its
 * robot.battery.uvp_threshold_v number. Only the raw EEPROM is read, never a copy of it on disk:
 * the threshold is a per-robot safety limit, and a writable copy could change it unnoticed.
 *
 * @param read_file Returns the file content, or nothing when the file cannot be read.
 */
UvpThreshold ResolveUvpThreshold(
  const std::optional<float> & param_threshold, const std::string & hat_eeprom_path,
  const std::function<std::optional<std::string>(const std::string &)> & read_file);

std::optional<std::string> ReadTextFile(const std::string & path);

}  // namespace husarion_ugv_battery

#endif  // HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_PROTECTION_UVP_PARAMETERS_HPP_
