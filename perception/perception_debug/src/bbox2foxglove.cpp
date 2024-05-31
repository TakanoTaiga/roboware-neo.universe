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

#include <cstdlib>
#include <memory>
#include <vector>

#include "perception_debug/bbox2foxglove_node.hpp"

namespace perceputil
{
void add_points(
  foxglove_msgs::msg::PointsAnnotation & input_annotation, const uint16_t & x, const uint16_t & y)
{
  auto point2 = foxglove_msgs::msg::Point2();
  point2.x = (double)x;
  point2.y = (double)y;
  input_annotation.points.push_back(point2);
}

foxglove_msgs::msg::Color rgba2color(
  const double & r, const double & g, const double & b, const double & a)
{
  auto color_msg = foxglove_msgs::msg::Color();
  color_msg.r = r;
  color_msg.g = g;
  color_msg.b = b;
  color_msg.a = a;
  return color_msg;
}
}  // namespace perceputil

namespace perception_debug
{
bbox2foxgloveNode::bbox2foxgloveNode(const rclcpp::NodeOptions & node_option)
: rclcpp::Node("bbox_to_foxglove_node", node_option)
{
  pub_foxglove_2d_bbox_ =
    create_publisher<foxglove_msgs::msg::ImageAnnotations>("output/image_annotation", 0);
  sub_yolox_ex_2d_bbox_ = create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
    "input/image_annotation", 0,
    std::bind(&bbox2foxgloveNode::subscriber_callback, this, std::placeholders::_1));
}

void bbox2foxgloveNode::subscriber_callback(const bboxes_ex_msgs::msg::BoundingBoxes msg)
{
  auto pub_msg = foxglove_msgs::msg::ImageAnnotations();

  for (const auto box : msg.bounding_boxes) {
    auto annotation = foxglove_msgs::msg::PointsAnnotation();
    annotation.timestamp = msg.header.stamp;
    annotation.type = foxglove_msgs::msg::PointsAnnotation::LINE_LOOP;
    perceputil::add_points(annotation, box.xmin, box.ymin);
    perceputil::add_points(annotation, box.xmax, box.ymin);
    perceputil::add_points(annotation, box.xmax, box.ymax);
    perceputil::add_points(annotation, box.xmin, box.ymax);
    perceputil::add_points(annotation, box.xmin, box.ymin);
    annotation.thickness = 2.0;
    annotation.outline_color = perceputil::rgba2color(0.905, 0.298, 0.235, 1.0);
    annotation.fill_color = perceputil::rgba2color(0, 0, 0, 0);

    pub_msg.points.push_back(annotation);

    auto label = foxglove_msgs::msg::TextAnnotation();
    label.timestamp = msg.header.stamp;
    label.position.x = box.xmin;
    label.position.y = box.ymin;
    label.text = box.class_id + "(" + std::to_string(box.id) + ")";
    label.text_color = perceputil::rgba2color(1, 1, 1, 1);
    label.background_color = perceputil::rgba2color(0.905, 0.298, 0.235, 1.0);
    label.font_size = 30;

    pub_msg.texts.push_back(label);
  }

  pub_foxglove_2d_bbox_->publish(pub_msg);
}
}  // namespace perception_debug

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_debug::bbox2foxgloveNode)