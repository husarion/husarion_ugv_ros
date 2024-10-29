// Copyright (c) 2024 Husarion Sp. z o.o.
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

#include <memory>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DockPosePublisherNode : public rclcpp::Node
{
public:
  DockPosePublisherNode() : Node("pose_publisher_node")
  {
    declare_parameter("device_namespace", "main");
    std::string device_namespace = get_parameter("device_namespace").as_string();
    std::string robot_namespace = std::string(get_namespace()).substr(1);

    source_frame_ = robot_namespace + "/" + device_namespace +
                    "_wibotic_receiver_requested_pose_link";
    target_frame_ = robot_namespace + "/odom";

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "docking/dock_pose", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&DockPosePublisherNode::publishPose, this));
  }

private:
  void publishPose()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = target_frame_;
    pose_msg.pose.position.x = transform_stamped.transform.translation.x;
    pose_msg.pose.position.y = transform_stamped.transform.translation.y;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation = transform_stamped.transform.rotation;

    pose_publisher_->publish(pose_msg);
  }

  std::string target_frame_;
  std::string source_frame_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DockPosePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
