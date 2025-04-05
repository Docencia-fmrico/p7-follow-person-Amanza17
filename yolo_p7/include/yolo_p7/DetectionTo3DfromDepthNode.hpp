// Archivo: DetectionTo3DfromDepthNode.hpp
#ifndef DETECTION_TO_3D_FROM_DEPTH_NODE_HPP
#define DETECTION_TO_3D_FROM_DEPTH_NODE_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "depth_image_proc/depth_traits.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

namespace yolo_p7
{

class DetectionTo3DfromDepthNode : public rclcpp::Node
{
public:
  DetectionTo3DfromDepthNode();

private:
  void callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg);
  void callback_sync(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg);

  using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> detection_sub_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
  std::shared_ptr<image_geometry::PinholeCameraModel> model_;
  bool camera_info_received_;
};

}  // namespace yolo_p7

#endif  // DETECTION_TO_3D_FROM_DEPTH_NODE_HPP