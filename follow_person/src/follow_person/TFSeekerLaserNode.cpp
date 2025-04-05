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

#include "follow_person/TFSeekerLaserNode.hpp"


namespace follow_person
{

using namespace std::chrono_literals;
using std::placeholders::_1;

TFSeekerLaserNode::TFSeekerLaserNode()
: Node("follow_person_laser"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0)
{
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = create_wall_timer(
    50ms, std::bind(&TFSeekerLaserNode::control_cycle, this));
  subscriber_rep_ = create_subscription<geometry_msgs::msg::Vector3>(
    "repuls_vector", 10, std::bind(&TFSeekerLaserNode::callback_rep, this, _1));
}


void TFSeekerLaserNode::callback_rep(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  //RCLCPP_INFO(get_logger(), "Repulsive vector =  %f %f %f", msg->x, msg->y, msg->z);
  rep_x_ = msg->x;
  rep_y_ = msg->y;
  rep_z_ = msg->z;

  if (std::isnan(rep_x_)) {
    rep_x_ = 0.0;
  }
  if (std::isnan(rep_y_)) {
    rep_y_ = 0.0;
  }
  if (std::isnan(rep_z_)) {
    rep_z_ = 0.0;
  }
  //active_measure_ = true;
}

void
TFSeekerLaserNode::control_cycle()
{
  tf2::Stamped<tf2::Transform> cl2target;
  std::string error;

  if (tf_buffer_.canTransform("camera_link", "target", tf2::TimePointZero, &error)) {
    auto cl2target_msg = tf_buffer_.lookupTransform(
      "camera_link", "target", tf2::TimePointZero);

    tf2::fromMsg(cl2target_msg, cl2target);

    double x = cl2target.getOrigin().x() - rep_x_ * peso_repulsivo_;
    double y = cl2target.getOrigin().y() - rep_y_ * peso_repulsivo_;

    double angle = atan2(y, x);
    double dist = sqrt(x * x + y * y);


    /*
    if (cl2target.stamp_ > tf2::TimePoint(std::chrono::nanoseconds(static_cast<int64_t>(1 * 1e9)))) {

      RCLCPP_WARN(get_logger(), "Transform too old");
      angle = 0.0;
      dist = 1.0;
    }*/


    double vel_rot = 0.0;
    double vel_lin = 0.0;

    if (fabs(angle) > 0.5) {
        vel_rot = std::clamp(vrot_pid_.get_output(angle), -1.0, 1.0);
    } else {
        vel_rot = std::clamp(vrot_pid_.get_output(angle), -1.0, 1.0);
        vel_lin = std::clamp(vlin_pid_.get_output(dist - 1.0), -0.6, 0.6);
    }




    geometry_msgs::msg::Twist twist;
    twist.linear.x = vel_lin;
    twist.angular.z = vel_rot;

    vel_publisher_->publish(twist);

    if (fabs(angle) < 0.2 && dist < 1.3) {
      RCLCPP_INFO(get_logger(), "Pew Pew Madafakas");
    }
  } else {
    auto cmd_vel_msg_search = geometry_msgs::msg::Twist();
    cmd_vel_msg_search.angular.z = 0.6;
    vel_publisher_->publish(cmd_vel_msg_search);

  }
}



}  //  namespace follow_person
