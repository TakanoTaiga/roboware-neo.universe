// Copyright 2024 Hakoroboken
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

#ifndef DEPTH_MERGE_NODE_HPP_
#define DEPTH_MERGE_NODE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>

#include <bboxes_ex_msgs/msg/bounding_boxes.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace depth_merge_node
{
class DepthMergerNode : public rclcpp::Node
{
public:
  explicit DepthMergerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr sub_2d_bbox_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  sensor_msgs::msg::Image::SharedPtr image_cache;
  double fx, fy, cx, cy;
  std::string camera_frame_id;

  void image_subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cam_info_subscriber_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void bbox_subscriber_callback(const bboxes_ex_msgs::msg::BoundingBoxes msg);
};
}  // namespace depth_merge_node

#endif