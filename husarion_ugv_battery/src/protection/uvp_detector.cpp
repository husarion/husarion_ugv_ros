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

#include "husarion_ugv_battery/protection/uvp_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace husarion_ugv_battery
{

std::string UvpStateToString(const UvpState state)
{
  switch (state) {
    case UvpState::UNKNOWN:
      return "UNKNOWN";
    case UvpState::OK:
      return "OK";
    case UvpState::CHARGE_ASAP:
      return "CHARGE_ASAP";
    case UvpState::UVP:
      return "UVP";
  }
  return "INVALID";
}

namespace
{

std::string FormatFloat(const float value)
{
  if (std::isnan(value)) {
    return "nan";
  }
  std::ostringstream stream;
  stream << std::setprecision(9) << value;
  return stream.str();
}

float ParseFloat(const std::string & text)
{
  if (text == "nan") {
    return std::numeric_limits<float>::quiet_NaN();
  }
  std::size_t pos = 0;
  const float value = std::stof(text, &pos);
  if (pos != text.size()) {
    throw std::invalid_argument(text);
  }
  return value;
}

}  // namespace

std::string SerializeUvpLatch(const UvpLatch & latch)
{
  std::ostringstream out;
  out << "boot_id=" << latch.boot_id << "\n";
  out << "trip_id=" << latch.trip_id << "\n";
  out << "trip_boottime_ns=" << latch.trip_boottime_ns << "\n";
  out << "trip_stamp_ns=" << latch.trip_stamp_ns << "\n";
  out << "battery_count=" << latch.battery_uvp.size() << "\n";
  for (std::size_t i = 0; i < latch.battery_uvp.size(); ++i) {
    out << "battery_" << i << "=" << (latch.battery_uvp[i] ? "UVP" : "CLEAR") << ","
        << FormatFloat(latch.trip_voltage.at(i)) << "," << FormatFloat(latch.trip_current.at(i))
        << "\n";
  }
  return out.str();
}

UvpLatch ParseUvpLatch(const std::string & text)
{
  std::map<std::string, std::string> values;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    const auto separator = line.find('=');
    if (separator == std::string::npos) {
      throw std::runtime_error("Malformed latch line: " + line);
    }
    values[line.substr(0, separator)] = line.substr(separator + 1);
  }

  const auto get = [&values](const std::string & key) {
    const auto it = values.find(key);
    if (it == values.end()) {
      throw std::runtime_error("Latch has no " + key);
    }
    return it->second;
  };

  UvpLatch latch;
  try {
    latch.boot_id = get("boot_id");
    latch.trip_id = static_cast<std::uint32_t>(std::stoul(get("trip_id")));
    latch.trip_boottime_ns = std::stoll(get("trip_boottime_ns"));
    latch.trip_stamp_ns = std::stoll(get("trip_stamp_ns"));
    const auto battery_count = std::stoul(get("battery_count"));
    if (battery_count > 2) {
      throw std::runtime_error("Latch has " + std::to_string(battery_count) + " batteries");
    }
    for (std::size_t i = 0; i < battery_count; ++i) {
      std::istringstream fields(get("battery_" + std::to_string(i)));
      std::string state, voltage, current;
      std::getline(fields, state, ',');
      std::getline(fields, voltage, ',');
      std::getline(fields, current, ',');
      if (state != "UVP" && state != "CLEAR") {
        throw std::runtime_error("Latch has battery state '" + state + "'");
      }
      latch.battery_uvp.push_back(state == "UVP");
      latch.trip_voltage.push_back(ParseFloat(voltage));
      latch.trip_current.push_back(ParseFloat(current));
    }
  } catch (const std::logic_error & e) {
    throw std::runtime_error("Malformed latch value: " + std::string(e.what()));
  }
  return latch;
}

UvpDetector::UvpDetector(
  const UvpDetectorConfig & config, const std::size_t battery_count, const Clocks & clocks)
: config_(config), clocks_(clocks), start_ns_(clocks.steady_ns()), batteries_(battery_count)
{
  for (auto & battery : batteries_) {
    battery.last_valid_ns = start_ns_;
  }
}

bool UvpDetector::IsPlausible(const float voltage) const
{
  return !std::isnan(voltage) && voltage >= kMinPlausibleVoltage && voltage <= kMaxPlausibleVoltage;
}

void UvpDetector::FeedVoltage(
  const std::size_t battery_index, const float voltage, const bool temperature_valid)
{
  auto & battery = batteries_.at(battery_index);

  if (!temperature_valid || !IsPlausible(voltage)) {
    ++battery.invalid_reads;
    return;
  }

  battery.last_valid_ns = clocks_.steady_ns();
  battery.last_valid_voltage = voltage;
  battery.below_uvp_count = voltage < GetUvpThreshold() ? battery.below_uvp_count + 1 : 0;
  battery.below_asap_count = voltage < GetChargeAsapThreshold() ? battery.below_asap_count + 1 : 0;

  if (battery.state == UvpState::UNKNOWN) {
    SetBatteryState(battery_index, UvpState::OK);
  }

  Evaluate(battery_index);
}

void UvpDetector::FeedCurrent(const std::size_t battery_index, const float current)
{
  auto & battery = batteries_.at(battery_index);
  battery.last_current = current;

  // The current is read after the voltage, so a trip in this read takes its current here.
  if (battery.trip_current_pending) {
    battery.trip_current_pending = false;
    battery.trip_current = current;
    ++revision_;
  }
}

void UvpDetector::FeedCharging(const std::size_t battery_index, const bool charging)
{
  auto & battery = batteries_.at(battery_index);
  const auto now = clocks_.steady_ns();

  bool debounced = false;
  if (charging) {
    if (!battery.charging_since_ns.has_value()) {
      battery.charging_since_ns = now;
    }
    debounced = static_cast<double>(now - battery.charging_since_ns.value()) / 1e9 >=
                config_.charging_debounce_s;
  } else {
    // A flap restarts the debounce.
    battery.charging_since_ns.reset();
  }

  if (debounced != battery.charging) {
    battery.charging = debounced;
    ++revision_;
  }

  Evaluate(battery_index);
}

void UvpDetector::Evaluate(const std::size_t battery_index)
{
  auto & battery = batteries_.at(battery_index);
  if (battery.state == UvpState::UNKNOWN) {
    return;
  }

  if (battery.charging) {
    if (battery.state == UvpState::CHARGE_ASAP) {
      SetBatteryState(battery_index, UvpState::OK);
    } else if (
      battery.state == UvpState::UVP &&
      battery.last_valid_voltage >= GetUvpThreshold() + config_.clear_hysteresis) {
      SetBatteryState(battery_index, UvpState::OK);
    }
  }

  if (battery.state != UvpState::UVP && battery.below_uvp_count >= config_.consecutive_samples) {
    battery.trip_voltage = battery.last_valid_voltage;
    battery.trip_current = battery.last_current;
    battery.trip_current_pending = true;
    SetBatteryState(battery_index, UvpState::UVP);
    return;
  }

  // A charging battery cannot be told to charge.
  if (
    battery.state == UvpState::OK && !battery.charging &&
    battery.below_asap_count >= config_.consecutive_samples) {
    SetBatteryState(battery_index, UvpState::CHARGE_ASAP);
  }
}

void UvpDetector::SetBatteryState(const std::size_t battery_index, const UvpState state)
{
  auto & battery = batteries_.at(battery_index);
  if (battery.state == state) {
    return;
  }

  const auto robot_state_before = GetState();
  const bool uvp_change = battery.state == UvpState::UVP || state == UvpState::UVP;

  if (battery.state == UvpState::UVP || battery.state == UvpState::CHARGE_ASAP) {
    // A cleared state starts counting from scratch, so a stale count cannot trip it again at once.
    battery.below_uvp_count = 0;
    battery.below_asap_count = 0;
  }
  battery.state = state;
  ++revision_;

  if (uvp_change) {
    ++latch_revision_;
  }

  if (robot_state_before != UvpState::UVP && GetState() == UvpState::UVP) {
    ++trip_id_;
    trip_boottime_ns_ = clocks_.boottime_ns();
    trip_stamp_ns_ = clocks_.ros_ns();
  }
}

UvpState UvpDetector::GetState() const
{
  auto state = UvpState::UNKNOWN;
  for (const auto & battery : batteries_) {
    state = std::max(state, battery.state);
  }
  return state;
}

bool UvpDetector::IsCharging() const
{
  return !batteries_.empty() && std::all_of(
                                  batteries_.begin(), batteries_.end(),
                                  [](const BatteryDetector & battery) { return battery.charging; });
}

bool UvpDetector::IsBlind() const
{
  const auto now = clocks_.steady_ns();
  return std::any_of(batteries_.begin(), batteries_.end(), [&](const BatteryDetector & battery) {
    return static_cast<double>(now - battery.last_valid_ns) / 1e9 > config_.blind_timeout_s;
  });
}

double UvpDetector::GetSecondsSinceTrip() const
{
  if (trip_id_ == 0 || trip_boottime_ns_ == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(clocks_.boottime_ns() - trip_boottime_ns_) / 1e9;
}

UvpLatch UvpDetector::GetLatch(const std::string & boot_id) const
{
  UvpLatch latch;
  latch.boot_id = boot_id;
  latch.trip_id = trip_id_;
  latch.trip_boottime_ns = trip_boottime_ns_;
  latch.trip_stamp_ns = trip_stamp_ns_;
  for (const auto & battery : batteries_) {
    latch.battery_uvp.push_back(battery.state == UvpState::UVP);
    latch.trip_voltage.push_back(battery.trip_voltage);
    latch.trip_current.push_back(battery.trip_current);
  }
  return latch;
}

bool UvpDetector::RestoreLatch(
  const UvpLatch & latch, const std::string & boot_id, std::string & why)
{
  if (latch.boot_id != boot_id) {
    why = "the latch was written in another boot";
    return false;
  }
  // CLOCK_BOOTTIME starts again at a reboot, which a missing boot id cannot hide.
  if (latch.trip_boottime_ns > clocks_.boottime_ns()) {
    why = "the latch trip time is later than the time since boot";
    return false;
  }
  if (latch.battery_uvp.size() != batteries_.size()) {
    why = "the latch has " + std::to_string(latch.battery_uvp.size()) + " batteries, this robot " +
          std::to_string(batteries_.size());
    return false;
  }

  trip_id_ = latch.trip_id;
  trip_boottime_ns_ = latch.trip_boottime_ns;
  trip_stamp_ns_ = latch.trip_stamp_ns;
  for (std::size_t i = 0; i < batteries_.size(); ++i) {
    auto & battery = batteries_[i];
    battery.trip_voltage = latch.trip_voltage[i];
    battery.trip_current = latch.trip_current[i];
    if (latch.battery_uvp[i]) {
      battery.state = UvpState::UVP;
    }
  }
  ++revision_;
  return true;
}

}  // namespace husarion_ugv_battery
