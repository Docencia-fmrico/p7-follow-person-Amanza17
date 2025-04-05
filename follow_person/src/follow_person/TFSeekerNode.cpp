// Copyright 2024 Intelligent Robotics Lab
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

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "follow_person/TFSeekerNode.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

namespace follow_person
{

TFSeekerNode::TFSeekerNode()
: rclcpp_lifecycle::LifecycleNode("follow_person"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0)
{
  RCLCPP_INFO(get_logger(), "Constructor");
}

CallbackReturn
TFSeekerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TFSeekerNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating...");

  vel_publisher_->on_activate();

  timer_ = this->create_wall_timer(
    50ms, std::bind(&TFSeekerNode::control_cycle, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TFSeekerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");

  timer_ = nullptr;
  vel_publisher_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TFSeekerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  vel_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
TFSeekerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
TFSeekerNode::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Entering error state...");
  return CallbackReturn::SUCCESS;
}

void
TFSeekerNode::control_cycle()
{
  tf2::Stamped<tf2::Transform> cl2target;
  std::string error;

  if (tf_buffer_.canTransform("camera_link", "target", tf2::TimePointZero, &error)) {
    auto cl2target_msg = tf_buffer_.lookupTransform(
      "camera_link", "target", tf2::TimePointZero);

    tf2::fromMsg(cl2target_msg, cl2target);

    double x = cl2target.getOrigin().x();
    double y = cl2target.getOrigin().y();

    double angle = atan2(y, x);
    double dist = sqrt(x * x + y * y);

    vel_rot_ = 0.0;
    double vel_lin = 0.0;

    if (fabs(angle) > 0.5) {
      vel_rot_ = std::clamp(vrot_pid_.get_output(angle), -1.0, 1.0);
    } else {
      vel_rot_ = std::clamp(vrot_pid_.get_output(angle), -1.0, 1.0);
      vel_lin = std::clamp(vlin_pid_.get_output(dist - 1.0), -0.6, 0.6);
    }

    if (cl2target.stamp_ < tf2::TimePoint(std::chrono::nanoseconds(
        static_cast<int64_t>(this->now().nanoseconds() - 1e9)))) {
      RCLCPP_WARN(get_logger(), "Transform too old");
      vel_rot_ = 0.6;
      vel_lin = 0.0;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = vel_lin;
    twist.angular.z = vel_rot_;
    vel_publisher_->publish(twist);

    if (fabs(angle) < 0.2 && dist < 1.3) {
      RCLCPP_INFO(get_logger(), "Pew Pew Madafakas");
    }

  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error in TF camera_link -> target [" << error << "]");
    geometry_msgs::msg::Twist twist;
    twist.angular.z = 0.0;
    twist.linear.x = 0.0;
    vel_publisher_->publish(twist);
  }
}

}  // namespace follow_person
