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

#ifndef HUSARION_UGV_GAZEBO_GUI_E_STOP_HPP_
#define HUSARION_UGV_GAZEBO_GUI_E_STOP_HPP_

#include <string>

#include <gz/gui/Plugin.hh>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace husarion_ugv_gazebo
{

class EStop : public gz::gui::Plugin
{
  Q_OBJECT

public:
  EStop();
  ~EStop();
  void LoadConfig(const tinyxml2::XMLElement * plugin_elem) override;

  Q_INVOKABLE void ButtonPressed(bool checked);

  Q_PROPERTY(bool e_stop READ IsEStop WRITE SetEStop NOTIFY OnEStopChange)
  bool IsEStop() const { return e_stop_; }
  void SetEStop(bool value);

  Q_PROPERTY(QString ns READ GetNamespace WRITE SetNamespace NOTIFY OnNamespaceChange)
  QString GetNamespace() const { return QString::fromStdString(namespace_); }
  Q_INVOKABLE void SetNamespace(const QString & ns);

signals:
  void OnEStopChange();
  void OnNamespaceChange();

private:
  void OnEStopStatus(const std_msgs::msg::Bool::SharedPtr msg);

  static constexpr char kDefaultTopicName[] = "/hardware/e_stop";
  static constexpr char kDefaultResetSrvName[] = "/hardware/e_stop_reset";
  static constexpr char kDefaultTriggerSrvName[] = "/hardware/e_stop_trigger";
  const rclcpp::QoS e_stop_qos_ = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  bool e_stop_ = true;
  std::string namespace_ = "";
  std::string topic_name_ = kDefaultTopicName;
  std::string reset_srv_name_ = kDefaultResetSrvName;
  std::string trigger_srv_name_ = kDefaultTriggerSrvName;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_reset_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_trigger_client_;
};

}  // namespace husarion_ugv_gazebo

#endif  // HUSARION_UGV_GAZEBO_GUI_E_STOP_HPP_
