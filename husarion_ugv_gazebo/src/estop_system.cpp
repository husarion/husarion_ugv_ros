// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/imu.pb.h>
#include <gz/sim/components/Imu.hh>
#include <gz/transport/Node.hh>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>

#include "husarion_ugv_gazebo/estop_system.hpp"

struct jointData
{
  std::string name;
  double joint_position;
  double joint_velocity;
  double joint_effort;
  double joint_position_cmd;
  double joint_velocity_cmd;
  double joint_effort_cmd;
  bool is_actuated;
  gz::sim::Entity sim_joint;
  gz_ros2_control::GazeboSimSystemInterface::ControlMethod joint_control_method;
};

struct MimicJoint
{
  std::size_t joint_index;
  std::size_t mimicked_joint_index;
  double multiplier = 1.0;
  std::vector<std::string> interfaces_to_mimic;
};

class ImuData
{
public:
  std::string name{};
  std::string topicName{};
  gz::sim::Entity sim_imu_sensors_ = gz::sim::kNullEntity;
  std::array<double, 10> imu_sensor_data_;
  void OnIMU(const gz::msgs::IMU & _msg);
};

void ImuData::OnIMU(const gz::msgs::IMU & _msg)
{
  this->imu_sensor_data_[0] = _msg.orientation().x();
  this->imu_sensor_data_[1] = _msg.orientation().y();
  this->imu_sensor_data_[2] = _msg.orientation().z();
  this->imu_sensor_data_[3] = _msg.orientation().w();
  this->imu_sensor_data_[4] = _msg.angular_velocity().x();
  this->imu_sensor_data_[5] = _msg.angular_velocity().y();
  this->imu_sensor_data_[6] = _msg.angular_velocity().z();
  this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
  this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
  this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

class gz_ros2_control::GazeboSimSystemPrivate
{
public:
  GazeboSimSystemPrivate() = default;
  ~GazeboSimSystemPrivate() = default;
  size_t n_dof_;
  rclcpp::Time last_update_sim_time_ros_;
  std::vector<struct jointData> joints_;
  std::vector<std::shared_ptr<ImuData>> imus_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  gz::sim::EntityComponentManager * ecm;
  int * update_rate;
  gz::transport::Node node;
  std::vector<MimicJoint> mimic_joints_;
  double position_proportional_gain_;
};

namespace husarion_ugv_gazebo
{

CallbackReturn EStopSystem::on_init(const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  e_stop_active_ =
    hardware_interface::parse_bool(info_.hardware_parameters.at("e_stop_initial_state"));
  return CallbackReturn::SUCCESS;
}

CallbackReturn EStopSystem::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  SetupEStop();
  return hardware_interface::SystemInterface::on_configure(previous_state);
}

CallbackReturn EStopSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  PublishEStopStatus();
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn EStopSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type EStopSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (e_stop_active_) {
    return hardware_interface::return_type::OK;
  }

  return GazeboSimSystem::write(time, period);
}

void EStopSystem::SetupEStop()
{
  e_stop_publisher_ = nh_->create_publisher<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  e_stop_reset_service_ = nh_->create_service<TriggerSrv>(
    "hardware/e_stop_reset",
    std::bind(
      &EStopSystem::EStopResetCallback, this, std::placeholders::_1, std::placeholders::_2));

  e_stop_trigger_service_ = nh_->create_service<TriggerSrv>(
    "hardware/e_stop_trigger",
    std::bind(
      &EStopSystem::EStopTriggerCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void EStopSystem::PublishEStopStatus()
{
  std_msgs::msg::Bool e_stop_msg;
  e_stop_msg.data = e_stop_active_;
  e_stop_publisher_->publish(e_stop_msg);
}

void EStopSystem::EStopResetCallback(
  const TriggerSrv::Request::SharedPtr & /*request*/, TriggerSrv::Response::SharedPtr response)
{
  e_stop_active_ = false;
  response->success = true;
  response->message = "E-stop reset";
  PublishEStopStatus();
}

void EStopSystem::EStopTriggerCallback(
  const TriggerSrv::Request::SharedPtr & /*request*/, TriggerSrv::Response::SharedPtr response)
{
  e_stop_active_ = true;
  response->success = true;
  response->message = "E-stop triggered";
  PublishEStopStatus();
}

}  // namespace husarion_ugv_gazebo

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(husarion_ugv_gazebo::EStopSystem, gz_ros2_control::GazeboSimSystemInterface)
