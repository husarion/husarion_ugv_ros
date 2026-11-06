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

#ifndef HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_PROTECTION_PROTECTION_PUBLISHER_HPP_
#define HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_PROTECTION_PROTECTION_PUBLISHER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_msgs/msg/battery_protection.hpp"

#include "husarion_ugv_battery/battery/battery.hpp"
#include "husarion_ugv_battery/protection/uvp_detector.hpp"
#include "husarion_ugv_battery/protection/uvp_parameters.hpp"

namespace husarion_ugv_battery
{

using BatteryProtectionMsg = husarion_ugv_msgs::msg::BatteryProtection;

/**
 * @brief Runs the battery undervoltage protection on the batteries it is given: feeds the detector
 * from their reads, keeps the latch file, publishes battery/protection and adds the "Battery
 * protection" diagnostic task. Exists only when the protection is enabled.
 */
class ProtectionPublisher
{
public:
  struct Config
  {
    UvpParams params;
    UvpThreshold threshold;
    // False on the motor controller fallback, where the battery driver cannot see a charger.
    bool charging_detectable = true;
    double battery_timeout = 1.0;
    std::string boot_id;
  };

  ProtectionPublisher(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
    const std::vector<std::shared_ptr<Battery>> & batteries, const Config & config,
    const UvpDetector::Clocks & clocks);

  /**
   * @brief Takes the batteries' charging status of this tick, writes the latch when a UVP state
   * changed and publishes on a change or once a second. Call after the batteries updated.
   */
  void Update();

  BatteryProtectionMsg CreateMessage() const;
  void Diagnose(diagnostic_updater::DiagnosticStatusWrapper & status);

  bool ReactionsEnabled() const { return reactions_disabled_reason_.empty(); }
  const UvpDetector & GetDetector() const { return *detector_; }

  static std::string ReadBootId();
  static UvpDetector::Clocks SystemClocks(const rclcpp::Clock::SharedPtr & ros_clock);

private:
  void RestoreLatch();
  void WriteLatch();
  void LogStateChange(const UvpState previous, const UvpState current);
  rclcpp::Logger GetLogger() const;

  static constexpr std::int64_t kPublishPeriodNs = 1000000000;

  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::vector<std::shared_ptr<Battery>> batteries_;
  const Config config_;
  const UvpDetector::Clocks clocks_;
  std::unique_ptr<UvpDetector> detector_;
  std::string reactions_disabled_reason_;
  // Notes about the configuration and the latch, shown by the diagnostics for as long as the node
  // runs.
  std::vector<UvpNote> notes_;

  rclcpp::Publisher<BatteryProtectionMsg>::SharedPtr protection_pub_;
  std::uint64_t published_revision_ = 0;
  std::int64_t last_publish_ns_ = 0;
  bool published_ = false;
  std::uint64_t written_latch_revision_ = 0;
  UvpState last_state_ = UvpState::UNKNOWN;
};

}  // namespace husarion_ugv_battery

#endif  // HUSARION_UGV_BATTERY_HUSARION_UGV_BATTERY_PROTECTION_PROTECTION_PUBLISHER_HPP_
