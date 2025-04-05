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


#include <random>

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/rclcpp.hpp"

#include "follow_person/TFPublisherNode.hpp"


namespace follow_person
{

using namespace std::chrono_literals;
using std::placeholders::_1;

TFPublisherNode::TFPublisherNode()
: Node("tf_publisher")
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  subscriber_atrac_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "detection_3d", 10, std::bind(&TFPublisherNode::callback_atrac, this, _1));
}


void
TFPublisherNode::publish_tf()
{
  transform_.header.stamp = now();
  tf_broadcaster_->sendTransform(transform_);
}


void TFPublisherNode::callback_atrac(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{

  auto detection = msg->detections[0];  // Primera detección

  float atrac_x = detection.bbox.center.position.z;
  float atrac_y = -detection.bbox.center.position.x;
  float atrac_z = -detection.bbox.center.position.y;

  if (std::isnan(atrac_x)) {
    atrac_x = 0.0;
  }
  if (std::isnan(atrac_y)) {
    atrac_y = 0.0;
  }
  if (std::isnan(atrac_z)) {
    atrac_z = 0.0;
  }

  transform_.header.stamp = now();
  transform_.header.frame_id = "camera_link";
  transform_.child_frame_id = "target";

  transform_.transform.translation.x = atrac_x;
  transform_.transform.translation.y = atrac_y;
  transform_.transform.translation.z = atrac_z;

  RCLCPP_INFO(get_logger(), "TF at =  %f %f %f", atrac_x, atrac_y, atrac_z);
  
  publish_tf();

}


}  //  namespace follow_person
