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

#include "otos_driver/otos_driver_node.hpp"


namespace otos_driver
{
    otos_driver_node::otos_driver_node(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("otos_driver_node", node_option)
    {   
        sub_sensor_data = create_subscription<std_msgs::msg::String>(
            "input/sensor_data", 0, std::bind(&otos_driver_node::subscriber_callback, this, std::placeholders::_1));

        pub_current_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "output/current_pose", 0);
    }

    void otos_driver_node::subscriber_callback(const std_msgs::msg::String& msg)
    {
        std::stringstream ss(msg.data);
        std::string item;
        double values[3];
        int index = 0;

        while (std::getline(ss, item, ',') && index < 3) {
            values[index] = std::stod(item);
            index++;
        }

        auto result_message = geometry_msgs::msg::PoseStamped();
        result_message.header.frame_id = "base_link";
        result_message.header.stamp = get_clock()->now();
        result_message.pose.position.x = values[0];
        result_message.pose.position.y = values[1];
        result_message.pose.orientation = 
            rw_common_util::geometry::euler_to_rosquat(0.0, 0.0, values[2] * (M_PI / 180.0));
        pub_current_pose_->publish(result_message);
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(otos_driver::otos_driver_node)