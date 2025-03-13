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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_PHIDGET_IMU_SENSOR_PHIDGET_IMU_SENSOR_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_PHIDGET_IMU_SENSOR_PHIDGET_IMU_SENSOR_HPP_

#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "imu_filter_madgwick/imu_filter.h"
#include "imu_filter_madgwick/world_frame.h"
#include "phidgets_api/spatial.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "husarion_ugv_hardware_interfaces/phidgets_spatial_parameters.hpp"

using namespace std::placeholders;

namespace husarion_ugv_hardware_interfaces
{

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

/**
 * @brief Class that implements SensorInterface from ros2_control for PhidgetSpatial IMU
 */
class PhidgetImuSensor : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PhidgetImuSensor)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<StateInterface> export_state_interfaces() override;

  return_type read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override;

  static constexpr size_t kImuInterfacesSize = 10;
  static constexpr double KImuMagneticFieldUnknownValue = 1e300;
  static constexpr float G = 9.80665;

protected:
  /**
   * @brief Checks if the sensor name defined in the urdf matches the expected name.
   * @throw std::runtime_error If the sensor name does not match the expected name.
   */
  void CheckSensorName() const;

  /**
   * @brief Checks if the number of state interfaces defined in the urdf matches the expected size.
   * @throw std::runtime_error If the number of state interfaces does not match the expected size.
   */
  void CheckStatesSize() const;

  void SetInitialValues();
  void CheckInterfaces() const;
  void ReadObligatoryParams();
  void ReadCompassParams();
  void ReadMadgwickFilterParams();
  void CheckMadgwickFilterWorldFrameParam();
  bool IsParamDefined(const std::string & param_name) const;
  bool AreParamsDefined(const std::unordered_set<std::string> & params_names) const;

  void ConfigureCompassParams();
  void ConfigureHeating();
  void ConfigureMadgwickFilter();

  void SpatialDataCallback(
    const double acceleration[3], const double angular_rate[3], const double magnetic_field[3],
    const double timestamp);
  void SpatialAttachCallback();
  void SpatialDetachCallback();

  geometry_msgs::msg::Vector3 ParseMagnitude(const double magnetic_field[3]);
  geometry_msgs::msg::Vector3 ParseGyration(const double angular_rate[3]);
  geometry_msgs::msg::Vector3 ParseAcceleration(const double acceleration[3]);

  void InitializeMadgwickAlgorithm(
    const geometry_msgs::msg::Vector3 & mag_compensated,
    const geometry_msgs::msg::Vector3 & lin_acc, const rclcpp::Time & timestamp);
  void RestartMadgwickAlgorithm();

  bool IsIMUCalibrated(const geometry_msgs::msg::Vector3 & mag_compensated);

  bool IsVectorFinite(const geometry_msgs::msg::Vector3 & vec);

  void UpdateMadgwickAlgorithm(
    const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc,
    const geometry_msgs::msg::Vector3 & mag_compensated, const double dt);

  void UpdateMadgwickAlgorithmIMU(
    const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc,
    const double dt);

  void UpdateAccelerationAndGyrationStateValues(
    const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc);

  void UpdateAllStatesValues(
    const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc);

  void SetStateValuesToNans();

  void Calibrate();

  bool IsMagnitudeSynchronizedWithAccelerationAndGyration(
    const geometry_msgs::msg::Vector3 & mag_compensated);

  std::vector<double> imu_sensor_state_;

  rclcpp::Logger logger_{rclcpp::get_logger("PhidgetImuSensor")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  inline static const std::array<std::string, kImuInterfacesSize> kImuInterfacesNames = {
    "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z",
  };

  enum ImuMeasurements {
    orientation_x,
    orientation_y,
    orientation_z,
    orientation_w,
    angular_velocity_x,
    angular_velocity_y,
    angular_velocity_z,
    linear_acceleration_x,
    linear_acceleration_y,
    linear_acceleration_z
  };

  phidgets_spatial::Params params_;
  std::unique_ptr<phidgets::Spatial> spatial_;

  std::unique_ptr<ImuFilter> filter_;
  WorldFrame::WorldFrame world_frame_;
  bool imu_connected_ = false;

  bool imu_calibrated_ = false;
  std::mutex calibration_mutex_;
  std::condition_variable calibration_cv_;

  bool algorithm_initialized_ = false;
  rclcpp::Time last_spatial_data_timestamp_;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_PHIDGET_IMU_SENSOR_PHIDGET_IMU_SENSOR_HPP_
