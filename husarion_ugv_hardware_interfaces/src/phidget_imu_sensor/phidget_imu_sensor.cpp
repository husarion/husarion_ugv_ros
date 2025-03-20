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

#include "husarion_ugv_hardware_interfaces/phidget_imu_sensor/phidget_imu_sensor.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "imu_filter_madgwick/imu_filter.h"
#include "imu_filter_madgwick/stateless_orientation.h"
#include "imu_filter_madgwick/world_frame.h"
#include "phidgets_api/spatial.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace husarion_ugv_hardware_interfaces
{

CallbackReturn PhidgetImuSensor::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SensorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    CheckSensorName();
    CheckStatesSize();
    CheckInterfaces();
    SetInitialValues();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "An exception occurred while initializing: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PhidgetImuSensor::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    ReadObligatoryParams();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      logger_, "An exception occurred while reading the obligatory parameters: " << e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG_STREAM(logger_, "Phidgets Spatial IMU obligatory parameters: ");
  RCLCPP_DEBUG_STREAM(logger_, "\tserial_num: " << params_.serial);
  RCLCPP_DEBUG_STREAM(logger_, "\thub_port: " << params_.hub_port);
  RCLCPP_DEBUG_STREAM(logger_, "\tdata_interval_ms: " << params_.data_interval_ms << "ms");
  RCLCPP_DEBUG_STREAM(
    logger_, "\tcallback_delta_epsilon_ms " << params_.callback_delta_epsilon_ms << "ms");

  try {
    ReadMadgwickFilterParams();
    ConfigureMadgwickFilter();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      logger_, "An exception occurred while reading the Madgwick Filter parameters: " << e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG_STREAM(logger_, "Phidgets Spatial IMU Madgwick Filter parameters: ");
  RCLCPP_DEBUG_STREAM(logger_, "\tuse_mag: " << params_.use_mag);
  RCLCPP_DEBUG_STREAM(logger_, "\tgain: " << params_.gain);
  RCLCPP_DEBUG_STREAM(logger_, "\tzeta: " << params_.zeta << " rad/s");
  RCLCPP_DEBUG_STREAM(logger_, "\tmag_bias_x " << params_.mag_bias_x);
  RCLCPP_DEBUG_STREAM(logger_, "\tmag_bias_y " << params_.mag_bias_y);
  RCLCPP_DEBUG_STREAM(logger_, "\tmag_bias_z " << params_.mag_bias_z);
  RCLCPP_DEBUG_STREAM(logger_, "\tstateless " << params_.stateless);
  RCLCPP_DEBUG_STREAM(logger_, "\tremove_gravity_vector " << params_.remove_gravity_vector);

  return CallbackReturn::SUCCESS;
}

CallbackReturn PhidgetImuSensor::on_cleanup(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn PhidgetImuSensor::on_activate(const rclcpp_lifecycle::State &)
{
  rclcpp::NodeOptions ros_interface_options;

  RCLCPP_DEBUG_STREAM(
    logger_, "Connecting to Phidgets Spatial - serial: " << params_.serial
                                                         << ", hub port: " << params_.hub_port);

  try {
    if (!spatial_) {
      spatial_ = std::make_unique<phidgets::Spatial>(
        params_.serial, params_.hub_port, false,
        std::bind(&PhidgetImuSensor::SpatialDataCallback, this, _1, _2, _3, _4), nullptr,
        std::bind(&PhidgetImuSensor::SpatialAttachCallback, this),
        std::bind(&PhidgetImuSensor::SpatialDetachCallback, this));
    }

    imu_connected_ = true;
    RCLCPP_DEBUG_STREAM(
      logger_, "Successfully connected to the IMU sensor with serial number: "
                 << spatial_->getSerialNumber());
    Calibrate();

    spatial_->setDataInterval(params_.data_interval_ms);

    ConfigureCompassParams();
    ConfigureHeating();
  } catch (const phidgets::Phidget22Error & err) {
    RCLCPP_ERROR_STREAM(logger_, "An error occurred in the Spatial module: " << err.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PhidgetImuSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn PhidgetImuSensor::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn PhidgetImuSensor::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PhidgetImuSensor::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;

  for (size_t i = 0; i < kImuInterfacesSize; i++) {
    state_interfaces.emplace_back(
      info_.sensors.at(0).name, info_.sensors.at(0).state_interfaces.at(i).name,
      &imu_sensor_state_.at(i));
  }

  return state_interfaces;
}

return_type PhidgetImuSensor::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!imu_connected_) {
    return return_type::ERROR;
  }
  return return_type::OK;
}

void PhidgetImuSensor::CheckSensorName() const
{
  if (!info_.sensors.size()) {
    throw std::runtime_error("Sensor is not defined in URDF!");
  }
}

void PhidgetImuSensor::CheckStatesSize() const
{
  if (info_.sensors.at(0).state_interfaces.size() != kImuInterfacesSize) {
    throw std::runtime_error(
      "Wrong number of interfaces defined: " +
      std::to_string(info_.sensors.at(0).state_interfaces.size()) + ", " +
      std::to_string(kImuInterfacesSize) + " expected.");
  }
}

void PhidgetImuSensor::CheckInterfaces() const
{
  const auto names_start_iter = kImuInterfacesNames.begin();
  const auto names_end_iter = kImuInterfacesNames.end();
  const auto states_iter = info_.sensors.at(0).state_interfaces.begin();
  auto compare_name_with_interface_info =
    [](const std::string & name, const hardware_interface::InterfaceInfo & interface_info) {
      return name == interface_info.name;
    };

  if (!std::equal(
        names_start_iter, names_end_iter, states_iter, compare_name_with_interface_info)) {
    throw std::runtime_error("Mismatch detected in the state interfaces' names defined.");
  }
}

void PhidgetImuSensor::ReadObligatoryParams()
{
  params_.serial = std::stoi(info_.hardware_parameters.at("serial"));
  params_.hub_port = std::stoi(info_.hardware_parameters.at("hub_port"));
  params_.data_interval_ms = std::stoi(info_.hardware_parameters.at("data_interval_ms"));
  params_.callback_delta_epsilon_ms =
    std::stoi(info_.hardware_parameters.at("callback_delta_epsilon_ms"));

  if (params_.callback_delta_epsilon_ms >= params_.data_interval_ms) {
    throw std::runtime_error(
      "Invalid configuration: Callback epsilon is larger than the data interval.");
  }
}

void PhidgetImuSensor::ReadCompassParams()
{
  params_.cc_mag_field = hardware_interface::stod(info_.hardware_parameters.at("cc_mag_field"));
  params_.cc_offset0 = hardware_interface::stod(info_.hardware_parameters.at("cc_offset0"));
  params_.cc_offset1 = hardware_interface::stod(info_.hardware_parameters.at("cc_offset1"));
  params_.cc_offset2 = hardware_interface::stod(info_.hardware_parameters.at("cc_offset2"));
  params_.cc_gain0 = hardware_interface::stod(info_.hardware_parameters.at("cc_gain0"));
  params_.cc_gain1 = hardware_interface::stod(info_.hardware_parameters.at("cc_gain1"));
  params_.cc_gain2 = hardware_interface::stod(info_.hardware_parameters.at("cc_gain2"));
  params_.cc_t0 = hardware_interface::stod(info_.hardware_parameters.at("cc_t0"));
  params_.cc_t1 = hardware_interface::stod(info_.hardware_parameters.at("cc_t1"));
  params_.cc_t2 = hardware_interface::stod(info_.hardware_parameters.at("cc_t2"));
  params_.cc_t3 = hardware_interface::stod(info_.hardware_parameters.at("cc_t3"));
  params_.cc_t4 = hardware_interface::stod(info_.hardware_parameters.at("cc_t4"));
  params_.cc_t5 = hardware_interface::stod(info_.hardware_parameters.at("cc_t5"));
}

void PhidgetImuSensor::ReadMadgwickFilterParams()
{
  params_.gain = hardware_interface::stod(info_.hardware_parameters.at("gain"));
  params_.zeta = hardware_interface::stod(info_.hardware_parameters.at("zeta"));
  params_.mag_bias_x = hardware_interface::stod(info_.hardware_parameters.at("mag_bias_x"));
  params_.mag_bias_y = hardware_interface::stod(info_.hardware_parameters.at("mag_bias_y"));
  params_.mag_bias_z = hardware_interface::stod(info_.hardware_parameters.at("mag_bias_z"));
  params_.use_mag = hardware_interface::parse_bool(info_.hardware_parameters.at("use_mag"));
  params_.stateless = hardware_interface::parse_bool(info_.hardware_parameters.at("stateless"));
  params_.remove_gravity_vector =
    hardware_interface::parse_bool(info_.hardware_parameters.at("remove_gravity_vector"));

  CheckMadgwickFilterWorldFrameParam();
}

void PhidgetImuSensor::CheckMadgwickFilterWorldFrameParam()
{
  const auto world_frame = info_.hardware_parameters.at("world_frame");

  if (world_frame == "ned") {
    world_frame_ = WorldFrame::NED;
  } else if (world_frame == "nwu") {
    world_frame_ = WorldFrame::NWU;
  } else if (world_frame == "enu") {
    world_frame_ = WorldFrame::ENU;
  } else {
    RCLCPP_WARN_STREAM(
      logger_, "The parameter 'world_frame' was set to an invalid value ("
                 << world_frame
                 << "). Valid values are ['enu', 'ned', 'nwu']. Setting to default value 'enu'.");
  }
}

void PhidgetImuSensor::SetInitialValues()
{
  imu_sensor_state_.resize(
    info_.sensors.at(0).state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
}

void PhidgetImuSensor::Calibrate()
{
  spatial_->zero();

  RCLCPP_INFO(
    logger_, "Calibrating IMU sensor please ensure the robot remains stationary for 2 seconds.");

  std::unique_lock<std::mutex> lock(calibration_mutex_);
  calibration_cv_.wait(lock, [this]() { return imu_calibrated_; });

  RCLCPP_INFO(logger_, "IMU sensor calibration completed.");
}

bool PhidgetImuSensor::IsParamDefined(const std::string & param_name) const
{
  return info_.hardware_parameters.find(param_name) != info_.hardware_parameters.end();
}

bool PhidgetImuSensor::AreParamsDefined(const std::unordered_set<std::string> & params_names) const
{
  for (const auto & param_name : params_names) {
    if (!IsParamDefined(param_name)) {
      return false;
    }
  }
  return true;
}

void PhidgetImuSensor::ConfigureCompassParams()
{
  if (AreParamsDefined(
        {"cc_mag_field", "cc_offset0", "cc_offset1", "cc_offset2", "cc_gain0", "cc_gain1",
         "cc_gain2", "cc_t0", "cc_t1", "cc_t2", "cc_t3", "cc_t4", "cc_t5"})) {
    ReadCompassParams();

    spatial_->setCompassCorrectionParameters(
      params_.cc_mag_field, params_.cc_offset0, params_.cc_offset1, params_.cc_offset2,
      params_.cc_gain0, params_.cc_gain1, params_.cc_gain2, params_.cc_t0, params_.cc_t1,
      params_.cc_t2, params_.cc_t3, params_.cc_t4, params_.cc_t5);
  } else {
    RCLCPP_INFO(
      logger_, "Compass correction parameters not found. Skipping compass configuration.");
  }
}

void PhidgetImuSensor::ConfigureHeating()
{
  if (IsParamDefined("heating_enabled")) {
    params_.heating_enabled =
      hardware_interface::stod(info_.hardware_parameters["heating_enabled"]);
    spatial_->setHeatingEnabled(params_.heating_enabled);
  } else {
    RCLCPP_INFO(logger_, "Heating configuration parameter not found. Skipping heating setup.");
  }
}

void PhidgetImuSensor::ConfigureMadgwickFilter()
{
  filter_ = std::make_unique<ImuFilter>();
  filter_->setWorldFrame(world_frame_);
  filter_->setAlgorithmGain(params_.gain);
  filter_->setDriftBiasGain(params_.zeta);
}

geometry_msgs::msg::Vector3 PhidgetImuSensor::ParseMagnitude(const double magnetic_field[3])
{
  geometry_msgs::msg::Vector3 mag_fld;

  if (magnetic_field[0] == KImuMagneticFieldUnknownValue) {
    mag_fld.x = std::numeric_limits<double>::quiet_NaN();
    mag_fld.y = std::numeric_limits<double>::quiet_NaN();
    mag_fld.z = std::numeric_limits<double>::quiet_NaN();
  } else {
    mag_fld.x = magnetic_field[0] * 1e-4 - params_.mag_bias_x;
    mag_fld.y = magnetic_field[1] * 1e-4 - params_.mag_bias_y;
    mag_fld.z = magnetic_field[2] * 1e-4 - params_.mag_bias_z;
  }

  return mag_fld;
}

geometry_msgs::msg::Vector3 PhidgetImuSensor::ParseGyration(const double angular_rate[3])
{
  geometry_msgs::msg::Vector3 ang_vel;

  ang_vel.x = angular_rate[0] * (M_PI / 180.0);
  ang_vel.y = angular_rate[1] * (M_PI / 180.0);
  ang_vel.z = angular_rate[2] * (M_PI / 180.0);
  return ang_vel;
}

geometry_msgs::msg::Vector3 PhidgetImuSensor::ParseAcceleration(const double acceleration[3])
{
  geometry_msgs::msg::Vector3 lin_acc;

  lin_acc.x = -acceleration[0] * G;
  lin_acc.y = -acceleration[1] * G;
  lin_acc.z = -acceleration[2] * G;
  return lin_acc;
}

void PhidgetImuSensor::InitializeMadgwickAlgorithm(
  const geometry_msgs::msg::Vector3 & mag_compensated, const geometry_msgs::msg::Vector3 & lin_acc,
  const rclcpp::Time & timestamp)
{
  geometry_msgs::msg::Quaternion init_q;

  if (!StatelessOrientation::computeOrientation(world_frame_, lin_acc, mag_compensated, init_q)) {
    throw std::runtime_error(
      "Couldn't determine gravity direction. The IMU sensor seems to be in free fall.");
  }

  filter_->setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

  last_spatial_data_timestamp_ = timestamp;
  algorithm_initialized_ = true;
}

void PhidgetImuSensor::RestartMadgwickAlgorithm()
{
  if (!filter_) {
    return;
  }

  const auto restarted_value = std::numeric_limits<double>::quiet_NaN();
  filter_->setOrientation(restarted_value, restarted_value, restarted_value, restarted_value);
}

bool PhidgetImuSensor::IsIMUCalibrated(const geometry_msgs::msg::Vector3 & mag_compensated)
{
  if (imu_calibrated_) {
    return true;
  }

  imu_calibrated_ = IsVectorFinite(mag_compensated);
  return imu_calibrated_;
}

bool PhidgetImuSensor::IsVectorFinite(const geometry_msgs::msg::Vector3 & vec)
{
  return std::isfinite(vec.x) && std::isfinite(vec.y) && std::isfinite(vec.z);
}

bool PhidgetImuSensor::IsMagnitudeSynchronizedWithAccelerationAndGyration(
  const geometry_msgs::msg::Vector3 & mag_compensated)
{
  return IsVectorFinite(mag_compensated);
}

void PhidgetImuSensor::SpatialDataCallback(
  const double acceleration[3], const double angular_rate[3], const double magnetic_field[3],
  const double timestamp)
{
  const auto mag_compensated = ParseMagnitude(magnetic_field);
  const auto ang_vel = ParseGyration(angular_rate);
  const auto lin_acc = ParseAcceleration(acceleration);

  // Skip the data callback when IMU is not calibrated
  if (!IsIMUCalibrated(mag_compensated)) {
    return;
  } else {
    std::lock_guard lock(calibration_mutex_);
    calibration_cv_.notify_one();
  }

  rclcpp::Time spatial_data_timestamp =
    rclcpp::Time(timestamp * 1e6);  // timestamp comes in milliseconds
  auto dt = (spatial_data_timestamp - last_spatial_data_timestamp_).seconds();
  last_spatial_data_timestamp_ = spatial_data_timestamp;

  // Wait for the a magnitude and an acceleration to initialize the algorithm
  if (
    !algorithm_initialized_ &&
    !IsMagnitudeSynchronizedWithAccelerationAndGyration(mag_compensated)) {
    return;
  }

  if (!algorithm_initialized_ || params_.stateless) {
    try {
      InitializeMadgwickAlgorithm(mag_compensated, lin_acc, spatial_data_timestamp);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(
        logger_, "An exception occurred while initializing the Madgwick algorithm: " << e.what());
    }
  }

  if (algorithm_initialized_ && !params_.stateless) {
    if (dt == 0.0) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 5000,
        "Time difference between acquired IMU data is 0, Madgwick Filter will not update the "
        "orientation!");
    }

    if (IsMagnitudeSynchronizedWithAccelerationAndGyration(mag_compensated) && params_.use_mag) {
      UpdateMadgwickAlgorithm(ang_vel, lin_acc, mag_compensated, dt);
    } else {
      UpdateMadgwickAlgorithmIMU(ang_vel, lin_acc, dt);
    }
  }

  UpdateAllStatesValues(ang_vel, lin_acc);
}

void PhidgetImuSensor::SpatialAttachCallback()
{
  RCLCPP_INFO(logger_, "IMU sensor has successfully attached and is now connected.");
  imu_connected_ = true;
  on_activate(rclcpp_lifecycle::State{});
}

void PhidgetImuSensor::SpatialDetachCallback()
{
  RCLCPP_WARN(
    logger_,
    "IMU sensor has detached! If the USB IMU cable has not been intentionally unplugged, please "
    "check the connection.");
  imu_connected_ = false;
  algorithm_initialized_ = false;
  SetStateValuesToNans();
  RestartMadgwickAlgorithm();
  on_deactivate(rclcpp_lifecycle::State{});
}

void PhidgetImuSensor::UpdateMadgwickAlgorithm(
  const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc,
  const geometry_msgs::msg::Vector3 & mag_compensated, const double dt)
{
  filter_->madgwickAHRSupdate(
    ang_vel.x, ang_vel.y, ang_vel.z, lin_acc.x, lin_acc.y, lin_acc.z, mag_compensated.x,
    mag_compensated.y, mag_compensated.z, dt);
}

void PhidgetImuSensor::UpdateMadgwickAlgorithmIMU(
  const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc,
  const double dt)
{
  filter_->madgwickAHRSupdateIMU(
    ang_vel.x, ang_vel.y, ang_vel.z, lin_acc.x, lin_acc.y, lin_acc.z, dt);
}

void PhidgetImuSensor::UpdateAccelerationAndGyrationStateValues(
  const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc)
{
  imu_sensor_state_[angular_velocity_x] = ang_vel.x;
  imu_sensor_state_[angular_velocity_y] = ang_vel.y;
  imu_sensor_state_[angular_velocity_z] = ang_vel.z;

  if (params_.remove_gravity_vector) {
    float gx, gy, gz;
    filter_->getGravity(gx, gy, gz);
    imu_sensor_state_[linear_acceleration_x] = lin_acc.x - gx;
    imu_sensor_state_[linear_acceleration_y] = lin_acc.y - gy;
    imu_sensor_state_[linear_acceleration_z] = lin_acc.y - gz;
  } else {
    imu_sensor_state_[linear_acceleration_x] = lin_acc.x;
    imu_sensor_state_[linear_acceleration_y] = lin_acc.y;
    imu_sensor_state_[linear_acceleration_z] = lin_acc.z;
  }
}

void PhidgetImuSensor::UpdateAllStatesValues(
  const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc)
{
  filter_->getOrientation(
    imu_sensor_state_[orientation_w], imu_sensor_state_[orientation_x],
    imu_sensor_state_[orientation_y], imu_sensor_state_[orientation_z]);

  UpdateAccelerationAndGyrationStateValues(ang_vel, lin_acc);
}

void PhidgetImuSensor::SetStateValuesToNans()
{
  std::fill(
    imu_sensor_state_.begin(), imu_sensor_state_.end(), std::numeric_limits<double>::quiet_NaN());
}

}  // namespace husarion_ugv_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  husarion_ugv_hardware_interfaces::PhidgetImuSensor, hardware_interface::SensorInterface)
