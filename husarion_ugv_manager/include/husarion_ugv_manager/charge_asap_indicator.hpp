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

#ifndef HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_CHARGE_ASAP_INDICATOR_HPP_
#define HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_CHARGE_ASAP_INDICATOR_HPP_

#include <cstdint>
#include <optional>

namespace husarion_ugv_manager
{

/**
 * @brief Decides when the lights manager shows the CHARGE_ASAP animation.
 *
 * The animation sits on the alert layer of the lights, above the battery animations and below the
 * error ones. That layer plays an animation once, so it is sent again every period for as long as
 * the battery protection reports CHARGE_ASAP with its reactions on. Stale protection data stops it.
 */
class ChargeAsapIndicator
{
public:
  // BatteryProtection STATE_CHARGE_ASAP.
  static constexpr std::uint8_t kStateChargeAsap = 2;
  // A little longer than the animation, so the alert layer never queues it twice.
  static constexpr std::int64_t kPeriodNs = 5000000000;
  static constexpr std::int64_t kStaleNs = 3000000000;

  void SetProtection(
    const std::uint8_t state, const bool reactions_enabled, const std::int64_t now_ns)
  {
    state_ = state;
    reactions_enabled_ = reactions_enabled;
    received_ns_ = now_ns;
  }

  // True when the animation is due now. Counts it as sent.
  bool Update(const std::int64_t now_ns)
  {
    const bool active = received_ns_.has_value() && now_ns - received_ns_.value() <= kStaleNs &&
                        state_ == kStateChargeAsap && reactions_enabled_;
    if (!active) {
      last_sent_ns_.reset();
      return false;
    }
    if (last_sent_ns_.has_value() && now_ns - last_sent_ns_.value() < kPeriodNs) {
      return false;
    }
    last_sent_ns_ = now_ns;
    return true;
  }

private:
  std::uint8_t state_ = 0;
  bool reactions_enabled_ = false;
  std::optional<std::int64_t> received_ns_;
  std::optional<std::int64_t> last_sent_ns_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_CHARGE_ASAP_INDICATOR_HPP_
