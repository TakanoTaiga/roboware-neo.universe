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

#ifndef THROWN_OBJECT_ESTIMATOR_NODE_HPP_
#define THROWN_OBJECT_ESTIMATOR_NODE_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rw_common_msgs/msg/transform_array.hpp>

namespace thrown_object_estimator
{

class thrown_object_estimator_node : public rclcpp::Node
{
public:
    explicit thrown_object_estimator_node(const rclcpp::NodeOptions & node_options);

private:
    rclcpp::Subscription<rw_common_msgs::msg::TransformArray>::SharedPtr sub_detection_result;

    void subscriber_callback(const rw_common_msgs::msg::TransformArray& msg);
};
} // namespace perception_debug

#endif