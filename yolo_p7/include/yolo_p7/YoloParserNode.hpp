// Copyright 2023 Intelligent Robotics Lab
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

#ifndef YOLO_P7__YOLODETECTIONNODE_HPP_
#define YOLO_P7__YOLODETECTIONNODE_HPP_

#include <memory>

#include "yolo_msgs/msg/detection_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace yolo_p7
{

class YoloParserNode : public rclcpp::Node
{
public:
  YoloParserNode();

private:
  int object_id_  = -1;
  bool same_object_ = false;
  int object_to_detect_id_ = 0;
  void detection_callback(const yolo_msgs::msg::DetectionArray::ConstSharedPtr & msg);

  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
};

}  // namespace camera

#endif  // YOLO_P7__YOLODETECTIONNODE_HPP_
