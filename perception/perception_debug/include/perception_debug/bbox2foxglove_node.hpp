// Copyright 2023 Hakoroboken
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

#ifndef BBOX2FOXGLOVE_NODE_HPP_
#define BBOX2FOXGLOVE_NODE_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <bboxes_ex_msgs/msg/bounding_boxes.hpp>
#include <foxglove_msgs/msg/image_annotations.hpp>

namespace perception_debug
{

class bbox2foxgloveNode : public rclcpp::Node
{
public:
    explicit bbox2foxgloveNode(const rclcpp::NodeOptions & node_options);

private:
    rclcpp::Publisher<foxglove_msgs::msg::ImageAnnotations>::SharedPtr pub_foxglove_2d_bbox_;
    rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr sub_yolox_ex_2d_bbox_;

    void subscriber_callback(const bboxes_ex_msgs::msg::BoundingBoxes msg);
};
} // namespace perception_debug

#endif