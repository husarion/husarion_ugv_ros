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
#include <chrono>
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

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_controller.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/robot_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_error_filter.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_e_stop.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_ros_interface.hpp"

namespace husarion_ugv_hardware_interfaces
{

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
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
  unsigned ReadRTPriority(const std::string & name, const unsigned default_priority);
  int ReadRTCPU(const std::string & name, const int default_cpu);
  virtual void ReadCANopenSettingsDriverCANIDs() = 0;

  void ReadInitializationActivationAttempts();
  void ReadParametersAndCreateRoboteqErrorFilter();
  void ReadDriverStatesUpdateFrequency();

  virtual void ConfigureGPIOController();  // virtual for mocking
  void ConfigureRobotDriver();
  virtual void DefineRobotDriver() = 0;
  virtual void ConfigureEStop();  // virtual for mocking
  void TriggerEStopServiceCall();
  void ResetEStop();

  void UpdateMotorsState();
  void UpdateDriverState();
  void UpdateEStopState();

  virtual void UpdateHwStates() = 0;
  // Returns whether motor states PDO data is currently timed out.
  virtual bool UpdateMotorsStateDataTimedOut() = 0;  // possible but needs changes in robot driver
  void UpdateCANopenResyncWatchdog(const bool data_timed_out);
  bool AreVelocityCommandsNearZero();

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
    std::string channel_1_motor;
    std::string channel_2_motor;
    DriverData data;
  };

  // The copy is made without a lock while the control loop may be writing the same data, so one
  // sample can mix two read cycles. That is harmless for a 1 Hz diagnostic, and a lock here would
  // put the diagnostic thread in the way of the control loop.
  virtual std::vector<DriverDiagnosticData> GetDriversDiagnosticData() = 0;

  /**
   * @brief Fills the system status task from the drivers' data. Never throws: a diagnostic tick
   * can race the hardware teardown, GetData() on a deinitialized driver throws, and an exception
   * escaping a diagnostic task terminates the whole node (bit us live on a Lynx).
   */
  void DiagnoseDriversStatus(
    diagnostic_updater::DiagnosticStatusWrapper & status, const std::string & robot_name);

  /**
   * @brief Fills the system errors task. The level comes from the error filter and from faults
   * the drivers report themselves. A single late PDO is a WARN: the filter escalates it to an
   * ERROR once it repeats. Never throws, see DiagnoseDriversStatus.
   */
  void DiagnoseDriversErrors(diagnostic_updater::DiagnosticStatusWrapper & status);

  // 10 C below the Over Temperature Limit (OTL 85 C) of the shipped Roboteq profile, at which the
  // controller stops the motors.
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
  // Kept apart from e_stop_, which the teardown resets while the diagnostics may still tick.
  std::shared_ptr<const EStopDiagnostics> e_stop_diagnostics_;
  // Same for the IO lines, which live in system_ros_interface_.
  std::shared_ptr<const IOStateDiagnosticsStore> io_state_diagnostics_;

  DrivetrainSettings drivetrain_settings_;
  CANopenSettings canopen_settings_;
  unsigned gpio_watchdog_sched_priority_ = Watchdog::kWatchdogSchedPriority;
  unsigned gpio_monitor_sched_priority_ = GPIOController::kMonitorSchedPriority;

  // Default cores of the RT threads, the same as the image's rt-tuning.env: the CAN chain on
  // RT_CAN_CORE=0, the control loop and the safety heartbeat next to it on RT_CONTROL_CORE=3.
  static constexpr int kDefaultCANCPU = 0;
  static constexpr int kDefaultControlCPU = 3;
  int gpio_watchdog_cpu_ = -1;
  int gpio_monitor_cpu_ = -1;

  std::unique_ptr<SystemROSInterface> system_ros_interface_;

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

  // Wall time between consecutive read() entries. A staleness warn cannot say
  // on its own whether the data was late or the reader was: at a 100 Hz
  // nominal cycle an age of 25 ms with a 10 ms cycle means the PDO stream
  // stalled, the same age with a 30 ms cycle means this loop missed wakeups
  // and read data that was fine when it arrived.
  float read_cycle_ms_ = 0.0F;
  timespec last_read_ts_{0, 0};
  rclcpp::Duration driver_states_update_period_{0, 0};

  // A PDO timeout persisting this long means the master stopped receiving while the bus can
  // still be healthy - re-initialization is the only recovery. Transients (SD stalls) are
  // 2-3 orders of magnitude shorter.
  static constexpr std::chrono::seconds pdo_timeout_resync_threshold_{30};
  static constexpr int resync_watchdog_exit_code_ = 74;  // EX_IOERR

  std::chrono::steady_clock::time_point pdo_timeout_since_;
  bool pdo_timeout_ongoing_ = false;

  // Counted by the control loop at the start of every motor state PDO timeout, read by the
  // diagnostics.
  std::atomic<std::uint32_t> pdo_timeout_events_{0};
  // The diagnostic thread's own record of the count it reported last.
  std::uint32_t pdo_timeout_events_reported_ = 0;

  // Set while the motor torque is off on request, which raises the Roboteq emergency stop fault.
  std::atomic_bool torque_disabled_by_request_{false};
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_UGV_SYSTEM_HPP_
