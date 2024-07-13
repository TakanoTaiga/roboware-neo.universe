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

#include "localization_debug/quat_to_euler_node.hpp"

namespace localization_debug
{
    quat_to_euler_node::quat_to_euler_node(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("quat_to_euler_node", node_option)
    {   
        sub_quat_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "input/quat", 0, std::bind(&quat_to_euler_node::callback_sub_euler, this, std::placeholders::_1));

        pub_euler_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "output/euler", 0);
    }

    void quat_to_euler_node::callback_sub_euler(const geometry_msgs::msg::PoseStamped& pose)
    {
        const auto euler_pose = rw_common_util::geometry::quat_to_euler(pose.pose.orientation);
        auto pub_msg = geometry_msgs::msg::Vector3Stamped();
        pub_msg.header = pose.header;
        pub_msg.vector.x = euler_pose.pitch * 57.295779513;
        pub_msg.vector.y = euler_pose.roll * 57.295779513;
        pub_msg.vector.z = euler_pose.yaw * 57.295779513;
        pub_euler_->publish(pub_msg);
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization_debug::quat_to_euler_node)