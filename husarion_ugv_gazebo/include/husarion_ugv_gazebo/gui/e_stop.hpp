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

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

namespace husarion_ugv_gazebo
{

class EStop : public ignition::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(QString ns READ GetNamespace WRITE SetNamespace NOTIFY ChangedNamespace)

public:
  EStop();
  virtual ~EStop();
  void LoadConfig(const tinyxml2::XMLElement * plugin_elem) override;
  Q_INVOKABLE QString GetNamespace() const;

public slots:
  void SetNamespace(const QString & ns);

signals:
  void ChangedNamespace();

protected slots:
  void ButtonPressed(bool pressed);

private:
  static constexpr char kDefaultEStopResetService[] = "/hardware/e_stop_reset";
  static constexpr char kDefaultEStopTriggerService[] = "/hardware/e_stop_trigger";

  std::string namespace_ = "";
  std::string e_stop_reset_service_ = kDefaultEStopResetService;
  std::string e_stop_trigger_service_ = kDefaultEStopTriggerService;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_reset_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_trigger_client_;
};
}  // namespace husarion_ugv_gazebo

#endif  // HUSARION_UGV_GAZEBO_GUI_E_STOP_HPP_
