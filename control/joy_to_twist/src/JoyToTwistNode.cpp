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

#include <memory>
#include <vector>
#include <cstdlib>

#include "joy_to_twist/JoyToTwistNode.hpp"

namespace joy_to_twist
{
    JoyToTwistNode::JoyToTwistNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("joy_to_twist_node", node_option)
    {   
        pub_cmd_pose_ = create_publisher<geometry_msgs::msg::Pose>(
            "output/cmd_pose", 0);
        sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
            "input/joy", 0, std::bind(&JoyToTwistNode::subscriber_callback, this, std::placeholders::_1));
    }

    void JoyToTwistNode::subscriber_callback(const sensor_msgs::msg::Joy& msg){
        auto cmd = geometry_msgs::msg::Pose();
        cmd.position.x = msg.axes[0] * -1.0;
        cmd.position.y = msg.axes[1];
        cmd.orientation = rw_common_util::geometry::euler_to_rosquat(0, 0, msg.axes[3] * 3.141592);
        pub_cmd_pose_->publish(cmd);
        
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(joy_to_twist::JoyToTwistNode)