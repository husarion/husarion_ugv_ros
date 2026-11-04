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

#ifndef HUSARION_UGV_TELEOP_HUSARION_UGV_TELEOP_GAMEPAD_DIAGNOSTICS_HPP_
#define HUSARION_UGV_TELEOP_HUSARION_UGV_TELEOP_GAMEPAD_DIAGNOSTICS_HPP_

#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace husarion_ugv_teleop
{

struct GamepadDiagnostic
{
  unsigned char level;
  std::string message;
};

/**
 * @brief Level and message of the gamepad diagnostic. A gamepad is optional, so a robot that never
 * had one is OK. Only a gamepad that was connected and then lost asks for attention, since someone
 * may have been driving with it.
 */
inline GamepadDiagnostic DiagnoseGamepad(const bool connected, const bool connected_before)
{
  if (connected) {
    return {diagnostic_msgs::msg::DiagnosticStatus::OK, "Gamepad connected."};
  }
  if (connected_before) {
    return {diagnostic_msgs::msg::DiagnosticStatus::WARN, "Gamepad disconnected."};
  }
  return {diagnostic_msgs::msg::DiagnosticStatus::OK, "No gamepad connected (optional)."};
}

}  // namespace husarion_ugv_teleop

#endif  // HUSARION_UGV_TELEOP_HUSARION_UGV_TELEOP_GAMEPAD_DIAGNOSTICS_HPP_
