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

#include "localization_debug/localizationDebugNode.hpp"

namespace localization_debug
{
    localizationDebugNode::localizationDebugNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("localization_debug_node", node_option)
    {   
        pub_maker_ = create_publisher<visualization_msgs::msg::Marker>(
            "output/maker", 0);

        marker_msg = visualization_msgs::msg::Marker();

        marker_msg.header.frame_id = declare_parameter<std::string>("frame_id" , "object");

        marker_msg.ns = declare_parameter<std::string>("marker.namespace" , "object");
        marker_msg.id = declare_parameter<int>("marker.id" , 0);
        marker_msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD; 

        marker_msg.scale.x = declare_parameter<double>("marker.scale" , 1.0);
        marker_msg.scale.y = marker_msg.scale.x;
        marker_msg.scale.z = marker_msg.scale.x;

        marker_msg.color.r = declare_parameter<double>("marker.color.r" , 1.0);
        marker_msg.color.g = declare_parameter<double>("marker.color.g" , 1.0);
        marker_msg.color.b = declare_parameter<double>("marker.color.b" , 1.0);
        marker_msg.color.a = declare_parameter<double>("marker.color.a" , 1.0);

        marker_msg.lifetime.sec = declare_parameter<double>("marker.lifetime.sec" , 0.0);
        marker_msg.lifetime.nanosec = declare_parameter<double>("marker.lifetime.nanosec" , 0.0);

        marker_msg.frame_locked = false;
 
        marker_msg.mesh_resource = "file://" + declare_parameter<std::string>("marker.meshpath" , "/path/to/object");
        marker_msg.mesh_use_embedded_materials = false;

        const auto period = declare_parameter<int>("publsih_rate_ms" , 1000);
        pub_timer_  = create_wall_timer(std::chrono::milliseconds(period), std::bind(&localizationDebugNode::timer_callback, this));

        RCLCPP_INFO_STREAM(get_logger(),  "Initialize Task Done");
    }

    void localizationDebugNode::timer_callback(){
        marker_msg.header.stamp = get_clock()->now();
        pub_maker_->publish(marker_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization_debug::localizationDebugNode)