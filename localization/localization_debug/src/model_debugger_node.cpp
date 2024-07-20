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

#include "localization_debug/model_debugger_node.hpp"

#include <fstream>


namespace localization_debug
{
    model_debugger_node::model_debugger_node(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("model_debugger_node", node_option)
    {   
        sub_pose = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/localization/lidarslam/current_pose", 0, std::bind(&model_debugger_node::callback_sub_pose, this, std::placeholders::_1));

        // pub_euler_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        //     "output/euler", 0);
    }

    void model_debugger_node::callback_sub_pose(const geometry_msgs::msg::PoseStamped& pose)
    {

        auto convertTimestampToDouble = [](builtin_interfaces::msg::Time time) {
            auto sec = time.sec; 
            auto nanosec = time.nanosec;
            constexpr double NANOSECONDS_IN_SECOND = 1000000000.0;
            return static_cast<double>(sec) + static_cast<double>(nanosec) / NANOSECONDS_IN_SECOND;
        };
        const auto dt = convertTimestampToDouble(pose.header.stamp) - convertTimestampToDouble(previous_pose.header.stamp);
        
        using rw_common_util::geometry::operator-;
        const auto dpose = pose.pose.position - previous_pose.pose.position;
        const auto dnorm = std::hypot(dpose.x, dpose.y, dpose.z);
        const auto speed = dnorm / dt;
        const auto dspped = speed - previous_speed;
        const auto acceration = dspped / dt;
        std::cout << speed << "," << acceration << std::endl;

        std::ofstream debug_csv_file;
        debug_csv_file.open("/Users/taiga/Desktop/_milisec.csv" , std::ios::app);
        debug_csv_file << speed << "," << acceration << std::endl;

        previous_pose = pose;
        previous_speed = speed;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization_debug::model_debugger_node)