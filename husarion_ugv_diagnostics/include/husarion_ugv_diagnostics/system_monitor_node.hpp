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

#ifndef HUSARION_UGV_DIAGNOSTICS_SYSTEM_MONITOR_NODE_HPP_
#define HUSARION_UGV_DIAGNOSTICS_SYSTEM_MONITOR_NODE_HPP_

#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include "husarion_ugv_msgs/msg/system_status.hpp"

#include "husarion_ugv_diagnostics/filesystem.hpp"
#include "husarion_ugv_diagnostics/system_monitor_parameters.hpp"
#include "husarion_ugv_diagnostics/types.hpp"

using namespace std::chrono_literals;

namespace husarion_ugv_diagnostics
{

class SystemMonitorNode : public rclcpp::Node
{
public:
  SystemMonitorNode(
    const std::string & node_name, FilesystemInterface::SharedPtr filesystem,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /**
   * @brief Retrieves the system parameters and generate system status object describing the current
   * system state.
   *
   * @return The system status.
   */
  SystemStatus GetSystemStatus() const;

  std::vector<float> GetCoresUsages() const;
  float GetCPUMeanUsage(const std::vector<float> & usages) const;
  float GetCPUTemperature() const;
  float GetRAMUsage() const;
  float GetDiskUsage() const;

  /**
   * @brief Converts a SystemStatus object to a SystemStatus message.
   *
   * This function takes a SystemStatus object and converts it into a SystemStatus message.
   * The resulting message can be used to publish the system status over a ROS topic.
   *
   * @param status The SystemStatus object to be converted.
   * @return The converted SystemStatus message.
   */
  husarion_ugv_msgs::msg::SystemStatus SystemStatusToMessage(const SystemStatus & status);

private:
  void TimerCallback();
  void DiagnoseSystem(diagnostic_updater::DiagnosticStatusWrapper & status);

  FilesystemInterface::SharedPtr filesystem_;
  diagnostic_updater::Updater diagnostic_updater_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<husarion_ugv_msgs::msg::SystemStatus>::SharedPtr system_status_publisher_;

  system_monitor::Params params_;
  std::shared_ptr<system_monitor::ParamListener> param_listener_;

  static constexpr char kTemperatureInfoFilename[] = "/sys/class/thermal/thermal_zone0/temp";
  static constexpr char kRootDirectory[] = "/";
};
}  // namespace husarion_ugv_diagnostics
#endif  // HUSARION_UGV_DIAGNOSTICS_SYSTEM_MONITOR_NODE_HPP_
