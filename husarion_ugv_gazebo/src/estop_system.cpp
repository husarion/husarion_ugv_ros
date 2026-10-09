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

#include "husarion_ugv_gazebo/estop_system.hpp"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

namespace husarion_ugv_gazebo
{

EStopSystem::~EStopSystem()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (owns_rclcpp_context_ && rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void EStopSystem::Configure(
  const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & /*event_mgr*/)
{
  const auto model = gz::sim::Model(entity);
  if (!model.Valid(ecm)) {
    throw std::runtime_error(
      "Error: Failed to initialize because [" + model.Name(ecm) +
      "] (Entity=" + std::to_string(entity) +
      ") is not a model. Please make sure that EStopSystem is attached to a valid model.");
  }

  ns_ = sdf->HasElement("namespace") ? sdf->Get<std::string>("namespace") : ns_;
  e_stop_active_ = sdf->HasElement("initial_state") ? sdf->Get<bool>("initial_state") : true;

  for (const auto & joint : model.Joints(ecm)) {
    const auto * type = ecm.Component<gz::sim::components::JointType>(joint);
    if (
      type != nullptr &&
      (type->Data() == sdf::JointType::CONTINUOUS || type->Data() == sdf::JointType::REVOLUTE)) {
      joint_entities_.push_back(joint);
    }
  }

  if (joint_entities_.empty()) {
    gzwarn << "EStopSystem found no continuous/revolute joints on model [" << model.Name(ecm)
           << "]; the E-stop will not halt any joint." << std::endl;
  }

  SetupROSInterface();
}

void EStopSystem::PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm)
{
  if (info.paused || !e_stop_active_) {
    return;
  }

  for (const auto & entity : joint_entities_) {
    ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(entity, {0.0});
  }
}

void EStopSystem::SetupROSInterface()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    owns_rclcpp_context_ = true;
  }

  node_ = std::make_shared<rclcpp::Node>("gz_estop", ns_);

  e_stop_publisher_ = node_->create_publisher<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Latch the initial state before the services go live, so a trigger/reset arriving the moment the
  // spin thread starts cannot race the first published value.
  PublishEStopStatus();

  e_stop_reset_service_ = node_->create_service<TriggerSrv>(
    "hardware/e_stop_reset",
    std::bind(
      &EStopSystem::EStopResetCallback, this, std::placeholders::_1, std::placeholders::_2));

  e_stop_trigger_service_ = node_->create_service<TriggerSrv>(
    "hardware/e_stop_trigger",
    std::bind(
      &EStopSystem::EStopTriggerCallback, this, std::placeholders::_1, std::placeholders::_2));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
}

void EStopSystem::PublishEStopStatus()
{
  BoolMsg e_stop_msg;
  e_stop_msg.data = e_stop_active_;
  e_stop_publisher_->publish(e_stop_msg);
}

void EStopSystem::EStopResetCallback(
  const TriggerSrv::Request::SharedPtr /*request*/, TriggerSrv::Response::SharedPtr response)
{
  e_stop_active_ = false;
  response->success = true;
  response->message = "E-stop reset";
  PublishEStopStatus();
}

void EStopSystem::EStopTriggerCallback(
  const TriggerSrv::Request::SharedPtr /*request*/, TriggerSrv::Response::SharedPtr response)
{
  e_stop_active_ = true;
  response->success = true;
  response->message = "E-stop triggered";
  PublishEStopStatus();
}

}  // namespace husarion_ugv_gazebo

GZ_ADD_PLUGIN(
  husarion_ugv_gazebo::EStopSystem, gz::sim::System,
  husarion_ugv_gazebo::EStopSystem::ISystemConfigure,
  husarion_ugv_gazebo::EStopSystem::ISystemPreUpdate)
