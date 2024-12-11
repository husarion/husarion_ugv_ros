// Copyright 2024 Husarion sp. z o.o.
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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_PANTHER_SYSTEM_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_PANTHER_SYSTEM_HPP_

#include <string>
#include <vector>

#include "husarion_ugv_hardware_interfaces/robot_system/ugv_system.hpp"

namespace husarion_ugv_hardware_interfaces
{

/**
 * @brief Class that implements UGVSystem for Panther robot
 */
class PantherSystem : public UGVSystem
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PantherSystem)

  PantherSystem() : UGVSystem(kJointOrder) {}

  ~PantherSystem() = default;

protected:
  void ReadCANopenSettingsDriverCANIDs() override;

  virtual void DefineRobotDriver() override;  // virtual for testing

  void UpdateHwStates() override;
  void UpdateMotorsStateDataTimedOut() override;

  void UpdateDriverStateMsg() override;
  void UpdateFlagErrors() override;
  void UpdateDriverStateDataTimedOut() override;

  std::vector<float> GetSpeedCommands() const;

  void DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status) override;
  void DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status) override;

  static const inline std::vector<std::string> kJointOrder = {"fl", "fr", "rl", "rr"};
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_PANTHER_SYSTEM_HPP_
