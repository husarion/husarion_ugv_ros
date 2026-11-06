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

#include "husarion_ugv_battery/protection/protection_publisher.hpp"

#include <time.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_battery/protection/uvp_detector.hpp"
#include "husarion_ugv_battery/protection/uvp_parameters.hpp"

namespace husarion_ugv_battery
{

namespace
{

unsigned char ToDiagnosticLevel(const UvpNoteLevel level)
{
  switch (level) {
    case UvpNoteLevel::ERROR:
      return diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    case UvpNoteLevel::WARN:
      return diagnostic_updater::DiagnosticStatusWrapper::WARN;
    default:
      return diagnostic_updater::DiagnosticStatusWrapper::OK;
  }
}

std::string BatteryKey(
  const std::size_t battery_count, const std::size_t battery, const std::string & name)
{
  if (battery_count == 1) {
    return "Battery " + name;
  }
  return "Battery " + std::to_string(battery + 1) + " " + name;
}

}  // namespace

ProtectionPublisher::ProtectionPublisher(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
  const std::vector<std::shared_ptr<Battery>> & batteries, const Config & config,
  const UvpDetector::Clocks & clocks)
: node_(node),
  diagnostic_updater_(diagnostic_updater),
  batteries_(batteries),
  config_(config),
  clocks_(clocks)
{
  UvpDetectorConfig detector_config;
  detector_config.uvp_threshold = config.threshold.value;
  detector_config.charge_asap_offset = config.params.charge_asap_offset;
  detector_config.consecutive_samples = static_cast<unsigned>(config.params.consecutive_samples);
  detector_config.clear_hysteresis = config.params.clear_hysteresis;
  detector_config.charging_debounce_s = config.params.charging_debounce_s;
  detector_config.blind_timeout_s = config.battery_timeout;
  detector_ = std::make_unique<UvpDetector>(detector_config, batteries_.size(), clocks_);

  notes_ = config.params.notes;
  notes_.insert(notes_.end(), config.threshold.notes.begin(), config.threshold.notes.end());

  if (config.params.stop_timeout_min == uvp::kReactionsOffStopTimeoutMin) {
    reactions_disabled_reason_ = uvp::kReactionsOffByTimeout;
  } else if (!config.charging_detectable) {
    reactions_disabled_reason_ = uvp::kReactionsOffNoCharging;
  }

  RestoreLatch();
  last_state_ = detector_->GetState();
  written_latch_revision_ = detector_->GetLatchRevision();

  for (std::size_t i = 0; i < batteries_.size(); ++i) {
    batteries_[i]->SetProtectionObservers(
      [this, i](const float voltage, const bool temperature_valid) {
        detector_->FeedVoltage(i, voltage, temperature_valid);
      },
      [this, i](const float current) { detector_->FeedCurrent(i, current); });
  }

  protection_pub_ = node->create_publisher<BatteryProtectionMsg>(
    "battery/protection", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  diagnostic_updater_->add("Battery protection", this, &ProtectionPublisher::Diagnose);

  RCLCPP_INFO_STREAM(
    GetLogger(), "Battery undervoltage protection enabled: UVP at "
                   << detector_->GetUvpThreshold() << " V (" << config.threshold.source
                   << "), charge ASAP at " << detector_->GetChargeAsapThreshold()
                   << " V, reactions "
                   << (ReactionsEnabled() ? "enabled" : reactions_disabled_reason_) << ".");
}

void ProtectionPublisher::Update()
{
  for (std::size_t i = 0; i < batteries_.size(); ++i) {
    const auto status = batteries_[i]->GetBatteryMsg().power_supply_status;
    detector_->FeedCharging(
      i, status == BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING ||
           status == BatteryStateMsg::POWER_SUPPLY_STATUS_FULL);
  }

  const auto state = detector_->GetState();
  if (state != last_state_) {
    LogStateChange(last_state_, state);
    last_state_ = state;
  }

  if (detector_->GetLatchRevision() != written_latch_revision_) {
    written_latch_revision_ = detector_->GetLatchRevision();
    WriteLatch();
  }

  const auto now = clocks_.steady_ns();
  if (
    !published_ || detector_->GetRevision() != published_revision_ ||
    now - last_publish_ns_ >= kPublishPeriodNs) {
    protection_pub_->publish(CreateMessage());
    published_ = true;
    published_revision_ = detector_->GetRevision();
    last_publish_ns_ = now;
  }
}

BatteryProtectionMsg ProtectionPublisher::CreateMessage() const
{
  BatteryProtectionMsg msg;
  msg.header.stamp = rclcpp::Time(clocks_.ros_ns());
  msg.state = static_cast<std::uint8_t>(detector_->GetState());
  for (std::size_t i = 0; i < detector_->GetBatteryCount(); ++i) {
    msg.battery_states.push_back(static_cast<std::uint8_t>(detector_->GetBatteryState(i)));
    msg.trip_voltage.push_back(detector_->GetTripVoltage(i));
    msg.trip_current.push_back(detector_->GetTripCurrent(i));
  }
  msg.reactions_enabled = ReactionsEnabled();
  msg.reactions_disabled_reason = reactions_disabled_reason_;
  msg.mode = config_.params.mode == uvp::Mode::SHUTDOWN ? BatteryProtectionMsg::MODE_SHUTDOWN
                                                        : BatteryProtectionMsg::MODE_STOP;
  msg.stop_timeout_min = static_cast<float>(config_.params.stop_timeout_min);
  msg.drive_window_s = config_.params.drive_window_s;
  msg.aux_off_at_trip = config_.params.aux_off_at_trip;
  msg.shutdown_hosts_timeout_s = config_.params.shutdown_hosts_timeout_s;
  msg.uvp_threshold = detector_->GetUvpThreshold();
  msg.charge_asap_threshold = detector_->GetChargeAsapThreshold();
  msg.threshold_source = config_.threshold.source;
  msg.trip_id = detector_->GetTripId();
  msg.trip_stamp = rclcpp::Time(detector_->GetTripStampNs());
  msg.seconds_since_trip = static_cast<float>(detector_->GetSecondsSinceTrip());
  msg.charging = detector_->IsCharging();
  return msg;
}

void ProtectionPublisher::Diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char level = diagnostic_updater::DiagnosticStatusWrapper::OK;
  std::string message;
  const auto raise = [&level, &message](const unsigned char new_level, const std::string & text) {
    if (message.empty() || new_level > level) {
      level = new_level;
      message = text;
    }
  };

  const auto state = detector_->GetState();
  const auto battery_count = detector_->GetBatteryCount();

  status.add("State", UvpStateToString(state));
  for (std::size_t i = 0; i < battery_count && battery_count > 1; ++i) {
    status.add(
      BatteryKey(battery_count, i, "state"), UvpStateToString(detector_->GetBatteryState(i)));
  }
  status.add("UVP threshold (V)", detector_->GetUvpThreshold());
  status.add("Charge ASAP threshold (V)", detector_->GetChargeAsapThreshold());
  status.add("Threshold source", config_.threshold.source);
  status.add("Mode", config_.params.mode == uvp::Mode::SHUTDOWN ? "SHUTDOWN" : "STOP");
  status.add("Stop timeout T (min)", config_.params.stop_timeout_min);
  status.add("Drive window (s)", config_.params.drive_window_s);
  status.add("AUX off at trip", config_.params.aux_off_at_trip ? "yes" : "no");
  status.add("Reactions", ReactionsEnabled() ? std::string("enabled") : reactions_disabled_reason_);
  status.add("Charging", detector_->IsCharging() ? "yes" : "no");
  status.add("Trip id", detector_->GetTripId());
  if (detector_->GetTripId() > 0) {
    status.add("Seconds since trip", detector_->GetSecondsSinceTrip());
  }
  for (std::size_t i = 0; i < battery_count; ++i) {
    if (!std::isnan(detector_->GetTripVoltage(i))) {
      status.add(BatteryKey(battery_count, i, "trip voltage (V)"), detector_->GetTripVoltage(i));
      status.add(BatteryKey(battery_count, i, "trip current (A)"), detector_->GetTripCurrent(i));
    }
    status.add(BatteryKey(battery_count, i, "invalid reads"), detector_->GetInvalidReads(i));
  }

  switch (state) {
    case UvpState::UVP:
      raise(
        diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Battery undervoltage, UVP latched.");
      break;
    case UvpState::CHARGE_ASAP:
      raise(
        diagnostic_updater::DiagnosticStatusWrapper::WARN,
        "Battery low, charge as soon as possible.");
      break;
    case UvpState::UNKNOWN:
      raise(diagnostic_updater::DiagnosticStatusWrapper::OK, "No valid battery voltage read yet.");
      break;
    default:
      raise(
        diagnostic_updater::DiagnosticStatusWrapper::OK, "Battery voltage above the thresholds.");
      break;
  }

  if (detector_->IsBlind()) {
    raise(
      diagnostic_updater::DiagnosticStatusWrapper::ERROR,
      "UVP blind: no valid battery voltage read for longer than the battery timeout.");
  }

  if (!ReactionsEnabled()) {
    raise(diagnostic_updater::DiagnosticStatusWrapper::WARN, reactions_disabled_reason_);
  }

  for (std::size_t i = 0; i < notes_.size(); ++i) {
    status.add("Note " + std::to_string(i + 1), notes_[i].text);
    if (notes_[i].level != UvpNoteLevel::INFO) {
      raise(ToDiagnosticLevel(notes_[i].level), notes_[i].text);
    }
  }

  status.summary(level, message);
}

void ProtectionPublisher::RestoreLatch()
{
  if (config_.params.latch_path.empty()) {
    return;
  }

  const auto content = ReadTextFile(config_.params.latch_path);
  if (!content.has_value()) {
    return;
  }

  std::string why;
  try {
    const auto latch = ParseUvpLatch(content.value());
    if (detector_->RestoreLatch(latch, config_.boot_id, why)) {
      RCLCPP_WARN_STREAM(
        GetLogger(), "Restored the battery protection latch: state "
                       << UvpStateToString(detector_->GetState()) << ", trip id "
                       << detector_->GetTripId() << ".");
      return;
    }
  } catch (const std::runtime_error & e) {
    why = e.what();
    notes_.push_back({UvpNoteLevel::WARN, "Latch file ignored: " + why});
  }
  RCLCPP_INFO_STREAM(GetLogger(), "Battery protection latch not restored: " << why << ".");
}

void ProtectionPublisher::WriteLatch()
{
  if (config_.params.latch_path.empty()) {
    return;
  }

  // Written to a temporary file and renamed, so a crash mid-write never leaves half a latch.
  const auto temp_path = config_.params.latch_path + ".tmp";
  {
    std::ofstream file(temp_path, std::ios::trunc);
    file << SerializeUvpLatch(detector_->GetLatch(config_.boot_id));
    if (!file) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        GetLogger(), *rclcpp::Clock::make_shared(RCL_STEADY_TIME), 60000,
        "Failed to write the battery protection latch to " << temp_path);
      return;
    }
  }
  if (std::rename(temp_path.c_str(), config_.params.latch_path.c_str()) != 0) {
    RCLCPP_ERROR_STREAM(
      GetLogger(), "Failed to move the battery protection latch to " << config_.params.latch_path);
  }
}

void ProtectionPublisher::LogStateChange(const UvpState previous, const UvpState current)
{
  if (current == UvpState::UVP) {
    std::string voltages;
    for (std::size_t i = 0; i < detector_->GetBatteryCount(); ++i) {
      voltages += (i ? ", " : "") + std::to_string(detector_->GetTripVoltage(i));
    }
    RCLCPP_ERROR_STREAM(
      GetLogger(), "Battery undervoltage protection tripped (trip id "
                     << detector_->GetTripId() << ", voltage at trip " << voltages
                     << " V, threshold " << detector_->GetUvpThreshold() << " V). Reactions "
                     << (ReactionsEnabled() ? "enabled" : reactions_disabled_reason_) << ".");
  } else if (current == UvpState::CHARGE_ASAP) {
    RCLCPP_WARN_STREAM(
      GetLogger(), "Battery voltage below " << detector_->GetChargeAsapThreshold()
                                            << " V, charge as soon as possible.");
  } else {
    RCLCPP_INFO_STREAM(
      GetLogger(), "Battery protection state " << UvpStateToString(previous) << " -> "
                                               << UvpStateToString(current) << ".");
  }
}

rclcpp::Logger ProtectionPublisher::GetLogger() const
{
  if (auto node = node_.lock()) {
    return node->get_logger();
  }
  return rclcpp::get_logger("battery_protection");
}

std::string ProtectionPublisher::ReadBootId()
{
  auto boot_id = ReadTextFile("/proc/sys/kernel/random/boot_id").value_or("");
  while (!boot_id.empty() && (boot_id.back() == '\n' || boot_id.back() == '\0')) {
    boot_id.pop_back();
  }
  return boot_id;
}

UvpDetector::Clocks ProtectionPublisher::SystemClocks(const rclcpp::Clock::SharedPtr & ros_clock)
{
  UvpDetector::Clocks clocks;
  clocks.steady_ns = []() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
  };
  clocks.boottime_ns = []() {
    timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
  };
  clocks.ros_ns = [ros_clock]() { return ros_clock->now().nanoseconds(); };
  return clocks;
}

}  // namespace husarion_ugv_battery
