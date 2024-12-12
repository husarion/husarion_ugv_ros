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

#include "husarion_ugv_manager/plugins/condition/check_joy_msg.hpp"

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

namespace husarion_ugv_manager
{

BT::NodeStatus CheckJoyMsg::onTick(const JoyMsg::SharedPtr & last_msg)
{
  return (last_msg && checkTimeout(last_msg) && checkAxes(last_msg) && checkButtons(last_msg))
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

bool CheckJoyMsg::checkAxes(const JoyMsg::SharedPtr & last_msg)
{
  std::vector<float> expected_axes;
  getInput<std::vector<float>>("axes", expected_axes);

  if (expected_axes.empty()) {
    return true;
  }

  if (last_msg->axes.size() != expected_axes.size()) {
    RCLCPP_WARN_STREAM(
      this->logger(), GetLoggerPrefix(name())
                        << "Joy message has " << last_msg->axes.size()
                        << " axes, expected at least " << expected_axes.size());
    return false;
  }

  return std::equal(
    expected_axes.begin(), expected_axes.end(), last_msg->axes.begin(),
    [](float a, float b) { return std::fabs(a - b) <= std::numeric_limits<float>::epsilon(); });
}

bool CheckJoyMsg::checkButtons(const JoyMsg::SharedPtr & last_msg)
{
  std::vector<int> expected_buttons;
  getInput<std::vector<int>>("buttons", expected_buttons);

  if (expected_buttons.empty()) {
    return true;
  }

  if (last_msg->buttons.size() != expected_buttons.size()) {
    RCLCPP_WARN_STREAM(
      this->logger(), GetLoggerPrefix(name())
                        << "Joy message has " << last_msg->buttons.size()
                        << " buttons, expected at least " << expected_buttons.size());
    return false;
  }

  return std::equal(expected_buttons.begin(), expected_buttons.end(), last_msg->buttons.begin());
}

bool CheckJoyMsg::checkTimeout(const JoyMsg::SharedPtr & last_msg)
{
  double max_timeout;
  getInput<double>("timeout", max_timeout);

  rclcpp::Time now;
  {
    auto node = node_.lock();
    if (!node) {
      return false;
    }
    now = node->now();
  }

  const auto time_diff = now - last_msg->header.stamp;
  return (max_timeout <= 0.0) || time_diff < rclcpp::Duration::from_seconds(max_timeout);
}

}  // namespace husarion_ugv_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(husarion_ugv_manager::CheckJoyMsg, "CheckJoyMsg");
