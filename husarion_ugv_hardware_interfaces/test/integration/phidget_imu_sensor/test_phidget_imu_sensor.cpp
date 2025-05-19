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

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "husarion_ugv_hardware_interfaces/phidget_imu_sensor/phidget_imu_sensor.hpp"
#include "husarion_ugv_utils/test/test_utils.hpp"

class PhidgetImuSensorWrapper : public husarion_ugv_hardware_interfaces::PhidgetImuSensor
{
public:
  PhidgetImuSensorWrapper() {}

  void SetHardwareInfo(const hardware_interface::HardwareInfo & info)
  {
    hardware_interface::SensorInterface::info_ = info;
  }

  void CheckSensorName() const { PhidgetImuSensor::CheckSensorName(); }

  void CheckStatesSize() const { PhidgetImuSensor::CheckStatesSize(); }

  void CheckInterfaces() const { PhidgetImuSensor::CheckInterfaces(); }

  void ReadObligatoryParams() { PhidgetImuSensor::ReadObligatoryParams(); }

  void ReadMadgwickFilterParams() { PhidgetImuSensor::ReadMadgwickFilterParams(); }

  void ConfigureMadgwickFilter() { PhidgetImuSensor::ConfigureMadgwickFilter(); }

  void InitializeMadgwickAlgorithm(
    const geometry_msgs::msg::Vector3 & mag_compensated,
    const geometry_msgs::msg::Vector3 & lin_acc, const rclcpp::Time timestamp)
  {
    PhidgetImuSensor::InitializeMadgwickAlgorithm(mag_compensated, lin_acc, timestamp);
  }

  void SpatialDataCallback(
    const double acceleration[3], const double angular_rate[3], const double magnetic_field[3],
    const double timestamp)
  {
    PhidgetImuSensor::SpatialDataCallback(acceleration, angular_rate, magnetic_field, timestamp);
  }

  void SpatialAttachCallback() { PhidgetImuSensor::SpatialAttachCallback(); }

  void SpatialDetachCallback() { PhidgetImuSensor::SpatialDetachCallback(); }

  geometry_msgs::msg::Vector3 ParseMagnitude(const double magnetic_field[3])
  {
    return PhidgetImuSensor::ParseMagnitude(magnetic_field);
  }

  geometry_msgs::msg::Vector3 ParseGyration(const double angular_rate[3])
  {
    return PhidgetImuSensor::ParseGyration(angular_rate);
  }

  geometry_msgs::msg::Vector3 ParseAcceleration(const double acceleration[3])
  {
    return PhidgetImuSensor::ParseAcceleration(acceleration);
  }

  bool IsVectorFinite(const geometry_msgs::msg::Vector3 & vec)
  {
    return PhidgetImuSensor::IsVectorFinite(vec);
  }

  bool IsQuaternionFinite(const geometry_msgs::msg::Quaternion & quat)
  {
    return std::isfinite(quat.x) && std::isfinite(quat.y) && std::isfinite(quat.z) &&
           std::isfinite(quat.w);
  }

  bool IsIMUCalibrated(const geometry_msgs::msg::Vector3 & vec)
  {
    return PhidgetImuSensor::IsIMUCalibrated(vec);
  }

  geometry_msgs::msg::Quaternion GetQuaternion() const
  {
    geometry_msgs::msg::Quaternion q;
    q.x = imu_sensor_state_[PhidgetImuSensor::orientation_x];
    q.y = imu_sensor_state_[PhidgetImuSensor::orientation_y];
    q.z = imu_sensor_state_[PhidgetImuSensor::orientation_z];
    q.w = imu_sensor_state_[PhidgetImuSensor::orientation_w];
    return q;
  }

  geometry_msgs::msg::Vector3 GetAcceleration() const
  {
    geometry_msgs::msg::Vector3 accel;
    accel.x = imu_sensor_state_[PhidgetImuSensor::linear_acceleration_x];
    accel.y = imu_sensor_state_[PhidgetImuSensor::linear_acceleration_y];
    accel.z = imu_sensor_state_[PhidgetImuSensor::linear_acceleration_z];
    return accel;
  }

  geometry_msgs::msg::Vector3 GetGyration() const
  {
    geometry_msgs::msg::Vector3 gyro;
    gyro.x = imu_sensor_state_[PhidgetImuSensor::angular_velocity_x];
    gyro.y = imu_sensor_state_[PhidgetImuSensor::angular_velocity_y];
    gyro.z = imu_sensor_state_[PhidgetImuSensor::angular_velocity_z];
    return gyro;
  }
};

class TestPhidgetImuSensor : public testing::Test
{
public:
  TestPhidgetImuSensor();
  ~TestPhidgetImuSensor() {}

  void CreateResourceManagerFromUrdf(const std::string & urdf);

  hardware_interface::return_type ConfigurePhidgetImu();

  hardware_interface::return_type UnconfigurePhidgetImu();

  hardware_interface::return_type ActivatePhidgetImu();

  hardware_interface::return_type DeactivatePhidgetImu();

  hardware_interface::return_type ShutdownPhidgetImu();

  /**
   * @brief Creates and returns URDF as a string
   * @param param_map map with hardware parameters
   * @param list list of interfaces
   */
  static std::string BuildUrdf(
    const std::unordered_map<std::string, std::string> & param_map,
    const std::list<std::string> & interfaces_list);

  std::string GetDefaultPhidgetImuUrdf();

protected:
  /**
   * @brief Changes current state of the resource manager to the one set in parameters. It is
   * recommended to use wrapper functions
   * @param state_id
   * @param state_name
   */
  hardware_interface::return_type SetState(
    const std::uint8_t state_id, const std::string & state_name);

  hardware_interface::HardwareInfo CreateExampleInterfaces(
    const hardware_interface::HardwareInfo & info);

  hardware_interface::HardwareInfo CreateCorrectInterfaces(
    const hardware_interface::HardwareInfo & info);

  hardware_interface::HardwareInfo AddMadgwickParameters(
    const hardware_interface::HardwareInfo & info);

  std::list<hardware_interface::LoanedStateInterface> ClaimGoodStateInterfaces();

  inline static const std::string kPhidgetImuName = "imu";

  inline static const std::string kUrdfHeader = R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
<link name="base_link"/>
<ros2_control name="imu" type="sensor">
)";

  inline static const std::string kUrdfFooter = R"(</ros2_control>
</robot>
)";

  inline static const std::list<std::string> kImuInterfaces = {
    "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z",
  };

  inline static const std::unordered_map<std::string, std::string> kImuObligatoryParams{
    {"serial", "-1"},          {"hub_port", "0"},
    {"data_interval_ms", "8"}, {"callback_delta_epsilon_ms", "1"},
    {"gain", "0.00304"},       {"zeta", "0.00151"},
    {"mag_bias_x", "0.0"},     {"mag_bias_y", "0.0"},
    {"mag_bias_z", "0.0"},     {"use_mag", "true"},
    {"stateless", "false"},    {"remove_gravity_vector", "false"},
    {"world_frame", "enu"}};

  inline static const std::string kPluginName =
    "<plugin>husarion_ugv_hardware_interfaces/PhidgetImuSensor</plugin>";

  std::unique_ptr<PhidgetImuSensorWrapper> imu_sensor_;
  std::shared_ptr<hardware_interface::ResourceManager> rm_;
  std::shared_ptr<rclcpp::Node> node_ = std::make_shared<rclcpp::Node>("PhidgetIMUTest");
};

TestPhidgetImuSensor::TestPhidgetImuSensor()
{
  imu_sensor_ = std::make_unique<PhidgetImuSensorWrapper>();
}

void TestPhidgetImuSensor::CreateResourceManagerFromUrdf(const std::string & urdf)
{
  rm_ = std::make_unique<hardware_interface::ResourceManager>(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface());
}

hardware_interface::return_type TestPhidgetImuSensor::ConfigurePhidgetImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

hardware_interface::return_type TestPhidgetImuSensor::UnconfigurePhidgetImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

hardware_interface::return_type TestPhidgetImuSensor::ActivatePhidgetImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
}

hardware_interface::return_type TestPhidgetImuSensor::DeactivatePhidgetImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

hardware_interface::return_type TestPhidgetImuSensor::ShutdownPhidgetImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
    hardware_interface::lifecycle_state_names::FINALIZED);
}

std::string TestPhidgetImuSensor::BuildUrdf(
  const std::unordered_map<std::string, std::string> & param_map,
  const std::list<std::string> & interfaces_list)
{
  std::stringstream urdf;

  urdf << kUrdfHeader << "<hardware>" << std::endl << kPluginName;

  for (auto const & [key, val] : param_map) {
    urdf << "<param name=\"" << key << "\">" << val << "</param>" << std::endl;
  }

  urdf << "</hardware>" << std::endl;

  urdf << "<sensor name=\"imu\" >" << std::endl;

  for (auto const & val : interfaces_list) {
    urdf << "<state_interface name=\"" << val << "\"/>" << std::endl;
  }
  urdf << "</sensor>" << std::endl;

  urdf << kUrdfFooter;

  return urdf.str();
}

std::string TestPhidgetImuSensor::GetDefaultPhidgetImuUrdf()
{
  return BuildUrdf(kImuObligatoryParams, kImuInterfaces);
}
hardware_interface::return_type TestPhidgetImuSensor::SetState(
  const std::uint8_t state_id, const std::string & state_name)
{
  rclcpp_lifecycle::State state(state_id, state_name);
  return rm_->set_component_state(kPhidgetImuName, state);
}

hardware_interface::HardwareInfo TestPhidgetImuSensor::CreateExampleInterfaces(
  const hardware_interface::HardwareInfo & info)
{
  hardware_interface::HardwareInfo new_info(info);
  std::list<std::string> example_interfaces_names = {"state1", "state2", "state3", "state4",
                                                     "state5", "state6", "state7", "state8",
                                                     "state9", "state10"};
  for (const auto & interface_name : example_interfaces_names) {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = interface_name;
    new_info.sensors.front().state_interfaces.push_back(interface_info);
  }
  return new_info;
}

hardware_interface::HardwareInfo TestPhidgetImuSensor::CreateCorrectInterfaces(
  const hardware_interface::HardwareInfo & info)
{
  hardware_interface::HardwareInfo new_info(info);
  for (const auto & interface_name : TestPhidgetImuSensor::kImuInterfaces) {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = interface_name;
    new_info.sensors.front().state_interfaces.push_back(interface_info);
  }
  return new_info;
}

hardware_interface::HardwareInfo TestPhidgetImuSensor::AddMadgwickParameters(
  const hardware_interface::HardwareInfo & info)
{
  hardware_interface::HardwareInfo new_info(info);
  const std::vector<std::string> keys = {
    "gain",       "zeta",      "mag_bias_x", "mag_bias_y",
    "mag_bias_z", "stateless", "use_mag",    "remove_gravity_vector",
    "world_frame"};

  for (const auto & key : keys) {
    new_info.hardware_parameters[key] = kImuObligatoryParams.at(key);
  }

  return new_info;
}

std::list<hardware_interface::LoanedStateInterface> TestPhidgetImuSensor::ClaimGoodStateInterfaces()
{
  std::list<hardware_interface::LoanedStateInterface> list;
  for (const auto & interface_name : kImuInterfaces) {
    list.push_back(rm_->claim_state_interface(kPhidgetImuName + "/" + interface_name));
  }
  return list;
}

TEST_F(TestPhidgetImuSensor, CheckSensorName)
{
  hardware_interface::HardwareInfo info;
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->CheckSensorName(); }, std::runtime_error);

  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "imu";
  info.sensors.push_back(sensor_info);
  imu_sensor_->SetHardwareInfo(info);

  EXPECT_NO_THROW({ imu_sensor_->CheckSensorName(); });
}

TEST_F(TestPhidgetImuSensor, CheckStatesSize)
{
  hardware_interface::HardwareInfo info;
  info.sensors.push_back({});
  imu_sensor_->SetHardwareInfo(info);

  EXPECT_THROW({ imu_sensor_->CheckStatesSize(); }, std::runtime_error);

  info = CreateExampleInterfaces(info);

  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->CheckStatesSize(); });
}

TEST_F(TestPhidgetImuSensor, CheckInterfaces)
{
  hardware_interface::HardwareInfo info;
  info.sensors.push_back({});

  info = CreateExampleInterfaces(info);

  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->CheckInterfaces(); }, std::runtime_error);

  info.sensors.front().state_interfaces.clear();

  info = CreateCorrectInterfaces(info);

  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->CheckInterfaces(); });
}

TEST_F(TestPhidgetImuSensor, ReadObligatoryParams)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters = {{"param1", "value1"}, {"param2", "value2"}};
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->ReadObligatoryParams(); }, std::exception);

  info.hardware_parameters = TestPhidgetImuSensor::kImuObligatoryParams;
  info.hardware_parameters["callback_delta_epsilon_ms"] =
    info.hardware_parameters["data_interval_ms"];
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->ReadObligatoryParams(); }, std::exception);

  info.hardware_parameters = TestPhidgetImuSensor::kImuObligatoryParams;
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->ReadObligatoryParams(); });
}

TEST_F(TestPhidgetImuSensor, CheckMagnitudeWrongValueAndCalibration)
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  hardware_interface::HardwareInfo info;
  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "imu";
  info.sensors.push_back(sensor_info);
  info = CreateCorrectInterfaces(info);

  ASSERT_EQ(imu_sensor_->on_init(info), CallbackReturn::SUCCESS);

  double magnitude[3];
  magnitude[0] = husarion_ugv_hardware_interfaces::PhidgetImuSensor::KImuMagneticFieldUnknownValue;

  const auto wrong_magnitude_parsed = imu_sensor_->ParseMagnitude(magnitude);
  ASSERT_FALSE(imu_sensor_->IsVectorFinite(wrong_magnitude_parsed));
  ASSERT_FALSE(imu_sensor_->IsIMUCalibrated(wrong_magnitude_parsed));

  magnitude[0] = 0.0;
  magnitude[1] = 0.0;
  magnitude[2] = 0.0;
  const auto magnitude_parsed = imu_sensor_->ParseMagnitude(magnitude);
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(magnitude_parsed));
  ASSERT_TRUE(imu_sensor_->IsIMUCalibrated(magnitude_parsed));
}

TEST_F(TestPhidgetImuSensor, CheckCalibrationOnDataCallbackAndAlgorithmInitialization)
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  hardware_interface::HardwareInfo info;
  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "imu";
  info.sensors.push_back(sensor_info);
  info = CreateCorrectInterfaces(info);
  info = AddMadgwickParameters(info);

  ASSERT_EQ(imu_sensor_->on_init(info), CallbackReturn::SUCCESS);
  double magnitude[3], acceleration[3], gyration[3];
  magnitude[0] = husarion_ugv_hardware_interfaces::PhidgetImuSensor::KImuMagneticFieldUnknownValue;
  const auto fake_wrong_magnitude_parsed = imu_sensor_->ParseMagnitude(magnitude);

  ASSERT_FALSE(imu_sensor_->IsIMUCalibrated(fake_wrong_magnitude_parsed));
  imu_sensor_->SpatialDataCallback(acceleration, gyration, magnitude, 100.0);
  ASSERT_FALSE(imu_sensor_->IsIMUCalibrated(fake_wrong_magnitude_parsed));

  // Correct values read from sensor
  magnitude[0] = -0.09675;
  magnitude[1] = 0.0675;
  magnitude[2] = 0.0795;

  acceleration[0] = -0.029536;
  acceleration[1] = 0.01302;
  acceleration[2] = 1.00752;

  gyration[0] = 0.0;
  gyration[1] = 0.0;
  gyration[2] = 0.0;

  // Configure Algorithm after calibration
  imu_sensor_->ReadMadgwickFilterParams();
  imu_sensor_->ConfigureMadgwickFilter();
  imu_sensor_->SpatialDataCallback(acceleration, gyration, magnitude, 100.0);

  ASSERT_TRUE(imu_sensor_->IsQuaternionFinite(imu_sensor_->GetQuaternion()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetAcceleration()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetGyration()));
}

TEST_F(TestPhidgetImuSensor, CheckReconnection)
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  hardware_interface::HardwareInfo info;
  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "imu";
  info.sensors.push_back(sensor_info);
  info = CreateCorrectInterfaces(info);
  info = AddMadgwickParameters(info);

  ASSERT_EQ(imu_sensor_->on_init(info), CallbackReturn::SUCCESS);
  double magnitude[3], acceleration[3], gyration[3];
  magnitude[0] = husarion_ugv_hardware_interfaces::PhidgetImuSensor::KImuMagneticFieldUnknownValue;

  // Correct values read from sensor
  magnitude[0] = -0.09675;
  magnitude[1] = 0.0675;
  magnitude[2] = 0.0795;

  acceleration[0] = -0.029536;
  acceleration[1] = 0.01302;
  acceleration[2] = 1.00752;

  gyration[0] = 0.0;
  gyration[1] = 0.0;
  gyration[2] = 0.0;

  // Configure Algorithm after calibration
  imu_sensor_->ReadMadgwickFilterParams();
  imu_sensor_->ConfigureMadgwickFilter();
  imu_sensor_->SpatialDataCallback(acceleration, gyration, magnitude, 100.0);

  ASSERT_TRUE(imu_sensor_->IsQuaternionFinite(imu_sensor_->GetQuaternion()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetAcceleration()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetGyration()));

  imu_sensor_->SpatialDetachCallback();
  ASSERT_FALSE(imu_sensor_->IsQuaternionFinite(imu_sensor_->GetQuaternion()));
  ASSERT_FALSE(imu_sensor_->IsVectorFinite(imu_sensor_->GetAcceleration()));
  ASSERT_FALSE(imu_sensor_->IsVectorFinite(imu_sensor_->GetGyration()));

  imu_sensor_->SpatialAttachCallback();
  imu_sensor_->SpatialDataCallback(acceleration, gyration, magnitude, 300.0);

  ASSERT_TRUE(imu_sensor_->IsQuaternionFinite(imu_sensor_->GetQuaternion()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetAcceleration()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetGyration()));
}

TEST_F(TestPhidgetImuSensor, CheckInterfacesLoadedByResourceManager)
{
  CreateResourceManagerFromUrdf(GetDefaultPhidgetImuUrdf());

  EXPECT_EQ(rm_->sensor_components_size(), 1u);
  ASSERT_EQ(rm_->state_interface_keys().size(), 10u);

  EXPECT_TRUE(rm_->state_interface_exists("imu/orientation.x"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/orientation.y"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/orientation.z"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/orientation.w"));

  EXPECT_TRUE(rm_->state_interface_exists("imu/angular_velocity.x"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/angular_velocity.y"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/angular_velocity.z"));

  EXPECT_TRUE(rm_->state_interface_exists("imu/linear_acceleration.x"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/linear_acceleration.y"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/linear_acceleration.z"));
}

TEST_F(TestPhidgetImuSensor, CheckStatesInitialValues)
{
  using hardware_interface::LoanedStateInterface;
  using hardware_interface::return_type;

  CreateResourceManagerFromUrdf(GetDefaultPhidgetImuUrdf());

  ASSERT_EQ(ConfigurePhidgetImu(), return_type::OK);

  auto loaded_state_interfaces = ClaimGoodStateInterfaces();

  for (const auto & state_interface : loaded_state_interfaces) {
    EXPECT_TRUE(std::isnan(state_interface.get_optional().value()));
  }

  EXPECT_EQ(ShutdownPhidgetImu(), return_type::OK);
}

TEST_F(TestPhidgetImuSensor, CheckWrongConfigurationWithWrongParameters)
{
  using hardware_interface::return_type;

  const std::string robot_system_urdf_ = BuildUrdf({}, TestPhidgetImuSensor::kImuInterfaces);
  CreateResourceManagerFromUrdf(robot_system_urdf_);

  EXPECT_EQ(ConfigurePhidgetImu(), return_type::ERROR);
  EXPECT_EQ(ShutdownPhidgetImu(), return_type::OK);
}

TEST_F(TestPhidgetImuSensor, CheckReadAndConfigureRealSensor)
{
  using hardware_interface::LoanedStateInterface;
  using hardware_interface::return_type;

  CreateResourceManagerFromUrdf(GetDefaultPhidgetImuUrdf());
  EXPECT_EQ(ConfigurePhidgetImu(), return_type::OK);

  ASSERT_EQ(ActivatePhidgetImu(), return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto loaded_state_interfaces = ClaimGoodStateInterfaces();
  for (const auto & state_interface : loaded_state_interfaces) {
    EXPECT_TRUE(std::isfinite(state_interface.get_optional().value()));
  }

  EXPECT_EQ(UnconfigurePhidgetImu(), return_type::OK);
  EXPECT_EQ(ShutdownPhidgetImu(), return_type::OK);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
