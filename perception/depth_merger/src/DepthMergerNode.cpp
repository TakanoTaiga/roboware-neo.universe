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

#include "depth_merge_node/DepthMergerNode.hpp"

namespace depth_merge_node
{
DepthMergerNode::DepthMergerNode(const rclcpp::NodeOptions & node_option)
: rclcpp::Node("depth_merger_node", node_option)
{
  camera_frame_id = declare_parameter<std::string>("frame_id", "base_link");

  sub_2d_bbox_ = create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
    "input/bbox", 0,
    std::bind(&DepthMergerNode::bbox_subscriber_callback, this, std::placeholders::_1));
  sub_depth_image_ = create_subscription<sensor_msgs::msg::Image>(
    "input/depth_raw", 0,
    std::bind(&DepthMergerNode::image_subscriber_callback, this, std::placeholders::_1));
  sub_cam_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "input/camera_info", 0,
    std::bind(&DepthMergerNode::cam_info_subscriber_callback, this, std::placeholders::_1));

  pub_debug_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("debug/marker", 0);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void DepthMergerNode::image_subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  image_cache = msg;
}

void DepthMergerNode::cam_info_subscriber_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  fx = msg->k[0];
  fy = msg->k[4];
  cx = msg->k[2];
  cy = msg->k[5];
}

void DepthMergerNode::bbox_subscriber_callback(const bboxes_ex_msgs::msg::BoundingBoxes msg)
{
  if (image_cache == nullptr) {
    return;
  }

  const auto cv_image = cv_bridge::toCvShare(image_cache, image_cache->encoding);

  auto marker_array = visualization_msgs::msg::MarkerArray();
  marker_array.markers.clear();

  for (const auto & bbox : msg.bounding_boxes) {
    uint64_t depth_sum = 0;
    uint32_t count = 0;
    uint32_t err_count = 0;

    for (uint16_t y = bbox.ymin; y <= bbox.ymax; y++) {
      for (uint16_t x = bbox.xmin; x <= bbox.xmax; x++) {
        const auto depth_mm = cv_image->image.at<uint16_t>(y, x);
        if (depth_mm > 520 && depth_mm < 4000) {
          depth_sum += depth_mm;
          count++;
        } else {
          err_count++;
        }
      }
    }

    if (count == 0) {
      continue;
    }
    if ((float)err_count / (float)(count + err_count) > 0.3) {
      continue;
    }

    float depth_avg_m = (float)(depth_sum / count) / 1000.0f;

    float bbox_size_x = bbox.xmax - bbox.xmin;
    float bbox_size_y = bbox.ymax - bbox.ymin;
    float position_x = bbox.xmin + bbox_size_x / 2;
    float position_y = bbox.ymin + bbox_size_y / 2;

    auto transform = geometry_msgs::msg::TransformStamped();
    transform.header = image_cache->header;
    transform.header.frame_id = "realsense_link";
    transform.child_frame_id = bbox.class_id;
    transform.transform.translation.x = depth_avg_m;
    transform.transform.translation.y = -(position_x - cx) * depth_avg_m / fx;
    transform.transform.translation.z = -(position_y - cy) * depth_avg_m / fy;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform);

    auto marker = visualization_msgs::msg::Marker();
    marker.header = image_cache->header;
    marker.header.frame_id = "realsense_link";
    marker.ns = "bbox3d";
    marker.id = bbox.id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.y = bbox_size_x * depth_avg_m / fy;
    marker.scale.x = marker.scale.y;
    marker.scale.z = bbox_size_y * depth_avg_m / fy;
    marker.pose.position.x = transform.transform.translation.x;
    marker.pose.position.y = transform.transform.translation.y;
    marker.pose.position.z = transform.transform.translation.z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    marker_array.markers.push_back(marker);
  }

  pub_debug_marker_->publish(marker_array);
}
}  // namespace depth_merge_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depth_merge_node::DepthMergerNode)