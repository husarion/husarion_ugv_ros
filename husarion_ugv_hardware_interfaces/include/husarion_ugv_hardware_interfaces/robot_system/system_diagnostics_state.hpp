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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_DIAGNOSTICS_STATE_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_DIAGNOSTICS_STATE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <deque>
#include <utility>

namespace husarion_ugv_hardware_interfaces
{

// A torque change this driver asked the motor controllers for, until their fault flags confirm
// it. A torque disable raises the Roboteq emergency stop fault, which is not a fault then.
enum class TorqueRequest : std::uint8_t {
  NONE = 0,
  // A disable was written and the drivers do not report the emergency stop fault yet.
  DISABLE_PENDING,
  // The drivers report the emergency stop fault the disable caused.
  DISABLED,
  // An enable was written and the drivers still report the emergency stop fault.
  ENABLE_PENDING,
  // Like DISABLE_PENDING and DISABLED, for the latch a refused E-stop reset writes again.
  RELATCH_PENDING,
  RELATCHED,
};

// A torque request the flags do not confirm within this time did not take effect.
constexpr std::chrono::seconds kTorqueRequestTimeout{1};

/**
 * @brief What the system diagnostics keep between two reports. The control loop writes the
 * atomics, the diagnostic thread alone uses the rest.
 */
struct SystemDiagnosticsState
{
  // Longest period between two read() calls since the last status report, in us.
  std::atomic<std::uint32_t> read_period_max_us{0};
  // Used by read() only. on_activate asks it to start a new period through the flag, because
  // lifecycle transitions may run on another thread.
  timespec last_read_ts{0, 0};
  std::atomic_bool restart_read_period{false};

  // Motor state PDO timeouts counted by the drivers, as reported last by the errors task and as
  // added to the history by the status task.
  std::uint32_t pdo_timeouts_reported = 0;
  std::uint32_t pdo_timeouts_in_history = 0;
  // New timeouts per status report over the last hour.
  std::deque<std::pair<std::chrono::steady_clock::time_point, std::uint32_t>> pdo_timeout_history;

  std::atomic<TorqueRequest> torque_request{TorqueRequest::NONE};
  // Steady clock time of the request, written before torque_request.
  std::atomic<std::int64_t> torque_request_ns{0};
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_DIAGNOSTICS_STATE_HPP_
