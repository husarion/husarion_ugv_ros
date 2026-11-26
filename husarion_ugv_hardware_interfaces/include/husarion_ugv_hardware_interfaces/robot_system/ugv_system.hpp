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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_UGV_SYSTEM_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_UGV_SYSTEM_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "husarion_ugv_msgs/msg/battery_protection.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_controller.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/robot_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_error_filter.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_diagnostics_state.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_e_stop.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_ros_interface.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/uvp_e_stop_guard.hpp"

namespace husarion_ugv_hardware_interfaces
{

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using BatteryProtectionMsg = husarion_ugv_msgs::msg::BatteryProtection;
using CommandInterface = hardware_interface::CommandInterface;

/**
 * @brief Class that implements SystemInterface from ros2_control for Husarion UGV
 */
class UGVSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UGVSystem)
  UGVSystem(const std::vector<std::string> & joint_order)
  : SystemInterface(),
    joint_size_(joint_order.size()),
    joint_order_(joint_order),
    joints_names_sorted_(joint_size_)
  {
  }

  virtual ~UGVSystem() = default;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<StateInterface> export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & /* period */) override;
  return_type write(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override;

protected:
  void CheckJointSize() const;
  void SortAndCheckJointNames();
  void SetInitialValues();
  void CheckInterfaces() const;

  void ReadDrivetrainSettings();
  void ReadCANopenSettings();
  void ReadRTThreadPriorities();
  unsigned ReadRTPriority(
    const std::string & name, const unsigned default_priority,
    const bool above_control_loop = false);
  int ReadRTCPU(const std::string & name, const int default_cpu);
  virtual void ReadCANopenSettingsDriverCANIDs() = 0;

  void ReadUVPSettings();
  void ReadInitializationActivationAttempts();
  void ReadParametersAndCreateRoboteqErrorFilter();
  void ReadDriverStatesUpdateFrequency();

  virtual void ConfigureGPIOController();  // virtual for mocking
  void ConfigureRobotDriver();
  virtual void DefineRobotDriver() = 0;
  virtual void ConfigureEStop();  // virtual for mocking
  void DestroyComponents();
  void TriggerEStopServiceCall();
  void ResetEStop();

  void UpdateMotorsState();
  void UpdateDriverState();
  void UpdateEStopState();

  // The battery undervoltage protection, only when the robot description enables it.
  void ConfigureUVP();
  void ProtectionCB(const std::shared_ptr<const BatteryProtectionMsg> msg);
  // Returns true when it triggered the e-stop in this cycle.
  bool UpdateUVPGuard(const bool e_stop);
  void DiagnoseUVP(
    diagnostic_updater::DiagnosticStatusWrapper & status, unsigned char & level,
    std::string & message);

  virtual void UpdateHwStates() = 0;
  virtual void UpdateMotorsStateDataTimedOut() = 0;  // possible but needs changes in robot driver
  bool AreVelocityCommandsNearZero();
  // Whether a motor controller of this robot reports its emergency stop fault.
  bool IsRoboteqEStopSet();

  virtual void UpdateDriverStateMsg() = 0;
  virtual void UpdateFlagErrors() = 0;               // possible but needs changes in robot driver
  virtual void UpdateDriverStateDataTimedOut() = 0;  // possible but needs changes in robot driver

  void HandleRobotDriverWriteOperation(std::function<void()> write_operation);
  virtual std::vector<float> GetSpeedCommands() const = 0;

  void MotorTorqueEnable(const bool enable);

  virtual void DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;
  virtual void DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;
  void DiagnoseEStop(diagnostic_updater::DiagnosticStatusWrapper & status);

  // A copy of one motor driver's data, with the names the diagnostics show for it.
  struct DriverDiagnosticData
  {
    std::string name;
    std::string right_motor;
    std::string left_motor;
    DriverData data;
  };

  // A known race, accepted for now: the copy is made without a lock while the control loop may
  // write the same data, as the diagnostics did before. A lock would put the diagnostic thread in
  // the way of the control loop.
  virtual std::vector<DriverDiagnosticData> GetDriversDiagnosticData() = 0;

  /**
   * @brief Fills the system status task from the drivers' data.
   */
  void DiagnoseDriversStatus(
    diagnostic_updater::DiagnosticStatusWrapper & status, const std::string & robot_name);

  /**
   * @brief Fills the system errors task. The level comes from the error filter and from faults
   * the drivers report themselves. A single late PDO is a WARN: the filter escalates it to an
   * ERROR once it repeats.
   */
  void DiagnoseDriversErrors(diagnostic_updater::DiagnosticStatusWrapper & status);

  void UpdateReadCycleMax();

  // Called by UpdateFlagErrors() before the error filter update, with whether any driver reports
  // the emergency stop fault. Ends a torque request once the flags confirm it.
  void UpdateTorqueRequest(const bool emergency_stop_set);
  // True when every flag error of the driver comes from a torque disable this driver asked for.
  bool IsFlagErrorOnlyTorqueDisable(const DriverData & data) const;
  void SetTorqueRequest(const TorqueRequest request);
  void MarkRelatchAfterRefusedReset();

  // 10 C below the 85 C overheat limit of the Roboteq configuration the robots ship with, at which
  // the controller stops the motors.
  static constexpr float kHeatsinkTemperatureWarn = 75.0F;

  const size_t joint_size_;

  // Currently only velocity command mode is supported - although Roboteq driver support position
  // and torque mode, in 2.1 firmware both modes aren't really stable and safe.
  // In torque mode sometimes after killing the software motor moves and it generally isn't well
  // tuned. Position mode also isn't really stable (reacts abruptly to spikes). If updating the
  // firmware to 2.1a will solve these issues, it may be worth to enable other modes.
  std::vector<double> hw_commands_velocities_;

  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_efforts_;

  // Define expected joint order, so that it doesn't matter what order is defined in the URDF.
  // It is expected that the joint name should contain these specifiers.
  const std::vector<std::string> joint_order_;
  std::vector<std::string> joints_names_sorted_;

  std::shared_ptr<GPIOControllerInterface> gpio_controller_;
  std::shared_ptr<RobotDriverInterface> robot_driver_;
  std::shared_ptr<EStopInterface> e_stop_;
  // Kept apart from e_stop_, so the diagnostic tasks do not depend on the teardown joining the
  // diagnostic executor before it resets e_stop_.
  std::shared_ptr<const EStopDiagnostics> e_stop_diagnostics_;
  // Same for the IO lines, which live in system_ros_interface_.
  std::shared_ptr<const IOStateDiagnosticsStore> io_state_diagnostics_;
  SystemDiagnosticsState diagnostics_state_;

  DrivetrainSettings drivetrain_settings_;
  CANopenSettings canopen_settings_;
  unsigned gpio_watchdog_sched_priority_ = Watchdog::kWatchdogSchedPriority;
  unsigned gpio_monitor_sched_priority_ = GPIOController::kMonitorSchedPriority;

  static constexpr int kDefaultRTCPU = -1;
  // ros2_control's default controller manager thread priority
  static constexpr unsigned kDefaultControlLoopPriority = 50;
  int gpio_watchdog_cpu_ = -1;
  int gpio_monitor_cpu_ = -1;

  std::unique_ptr<SystemROSInterface> system_ros_interface_;
  // What the GPIO edge callback publishes through, cleared before system_ros_interface_ is reset.
  std::mutex ros_interface_for_callbacks_mtx_;
  SystemROSInterface * ros_interface_for_callbacks_ = nullptr;

  // Sometimes SDO errors can happen during initialization and activation of Roboteq drivers,
  // in these cases it is better to retry
  // Example errors:
  // SDO abort code 05040000 received on upload request of object 1000 (Device type) to
  // node 02: SDO protocol timed out
  // SDO abort code 05040000 received on upload request of sub-object 1018:01 (Vendor-ID) to
  // node 02: SDO protocol timed out
  unsigned max_roboteq_initialization_attempts_;
  unsigned max_roboteq_activation_attempts_;

  rclcpp::Logger logger_{rclcpp::get_logger("UGVSystem")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter_;

  std::shared_ptr<std::mutex> robot_driver_write_mtx_;

  rclcpp::Time next_driver_state_update_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Duration driver_states_update_period_{0, 0};

  bool uvp_enable_ = false;
  std::unique_ptr<UvpEStopGuard> uvp_guard_;
  // Written by the protection subscription, read by the control loop without blocking it.
  realtime_tools::RealtimeBuffer<UvpProtectionInput> uvp_input_;
  std::atomic<std::int64_t> uvp_last_msg_ns_{0};
  std::int64_t uvp_configured_ns_ = 0;

  // Callback groups of SystemROSInterface, 0 being the group of the other services.
  static constexpr unsigned kEStopTriggerCallbackGroup = 1;
  static constexpr unsigned kEStopResetCallbackGroup = 2;
  static constexpr unsigned kBatteryProtectionCallbackGroup = 3;

  static constexpr double kUVPTopicStaleTimeout = 3.0;
  // The battery driver starts some seconds after this driver, so a missing topic is only an error
  // after this long.
  static constexpr double kUVPTopicStartupGrace = 30.0;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_UGV_SYSTEM_HPP_
