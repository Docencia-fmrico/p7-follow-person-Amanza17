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


#ifndef FOLLOW_PERSON__TFSEEKERNODE_HPP_
#define FOLLOW_PERSON__TFSEEKERNODE_HPP_

#include "follow_person/PIDController.hpp"

#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace follow_person
{

class TFSeekerNode : public rclcpp::Node
{
public:
  TFSeekerNode();

private:
  void control_cycle();
  double vel_rot_ = 0.5;


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  PIDController vlin_pid_, vrot_pid_;
};

}  //  namespace follow_person

#endif  // FOLLOW_PERSON__TFSEEKERNODE_HPP_
