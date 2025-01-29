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

#include "husarion_ugv_gazebo/gui/e_stop.hpp"

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/plugin/Register.hh>

namespace husarion_ugv_gazebo
{

EStop::EStop() : gz::gui::Plugin()
{
  rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>("gz_estop_gui");

  e_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_name_, e_stop_qos_, std::bind(&EStop::OnEStopStatus, this, std::placeholders::_1));
  e_stop_reset_client_ = node_->create_client<std_srvs::srv::Trigger>(reset_srv_name_);
  e_stop_trigger_client_ = node_->create_client<std_srvs::srv::Trigger>(trigger_srv_name_);

  std::thread([this]() { rclcpp::spin(node_); }).detach();
}

EStop::~EStop() { rclcpp::shutdown(); }

void EStop::LoadConfig(const tinyxml2::XMLElement * plugin_elem)
{
  if (title.empty()) {
    title = "E-stop";
  }

  if (plugin_elem) {
    auto namespace_elem = plugin_elem->FirstChildElement("namespace");
    if (namespace_elem != nullptr && namespace_elem->GetText() != nullptr) {
      SetNamespace(namespace_elem->GetText());
    }
  }
}

void EStop::ButtonPressed(bool e_stop)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto client = e_stop ? e_stop_reset_client_ : e_stop_trigger_client_;

  if (!client->service_is_ready()) {
    ignwarn << "Unavailable service: " << (e_stop ? reset_srv_name_ : trigger_srv_name_)
            << std::endl;
    return;
  }
  auto result_future = client->async_send_request(request);

  try {
    const auto result = result_future.get();
    if (!result->success) {
      ignwarn << "Service call did not succeed: " << result->message << std::endl;
    }
  } catch (const std::exception & e) {
    ignerr << "Exception while waiting for service response: " << e.what() << std::endl;
  }
}

void EStop::SetNamespace(const QString & ns)
{
  namespace_ = ns.toStdString();

  topic_name_ = namespace_ + kDefaultTopicName;
  reset_srv_name_ = namespace_ + kDefaultResetSrvName;
  trigger_srv_name_ = namespace_ + kDefaultTriggerSrvName;

  e_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_name_, e_stop_qos_, std::bind(&EStop::OnEStopStatus, this, std::placeholders::_1));
  e_stop_reset_client_ = node_->create_client<std_srvs::srv::Trigger>(reset_srv_name_);
  e_stop_trigger_client_ = node_->create_client<std_srvs::srv::Trigger>(trigger_srv_name_);

  emit OnNamespaceChange();
  ignmsg << "Namespace changed to: " << namespace_ << std::endl;
}

void EStop::SetEStop(bool value)
{
  e_stop_ = value;
  emit OnEStopChange();
}

void EStop::OnEStopStatus(const std_msgs::msg::Bool::SharedPtr msg) { SetEStop(msg->data); }

}  // namespace husarion_ugv_gazebo

GZ_ADD_PLUGIN(husarion_ugv_gazebo::EStop, gz::gui::Plugin)
