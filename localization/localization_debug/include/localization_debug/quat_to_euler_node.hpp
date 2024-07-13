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

#ifndef QUAT_TO_EULER_NODE_HPP
#define QUAT_TO_EULER_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <rw_common_util/geometry.hpp>

namespace localization_debug
{
    class quat_to_euler_node : public rclcpp::Node
    {
    public:
        explicit quat_to_euler_node(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_euler_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_quat_;

        void callback_sub_euler(const geometry_msgs::msg::PoseStamped& pose);
    };
} // namespace localization_debug

#endif