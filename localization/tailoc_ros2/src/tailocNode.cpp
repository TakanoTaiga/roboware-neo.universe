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

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "tailoc_ros2/tailocNode.hpp"
#include "tailoc_ros2/ndt_cpp.hpp"

namespace tailoc_ros2
{
    tailocNode::tailocNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("tailoc_node", node_option)
    {   
        sub_laser_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 0, std::bind(&tailocNode::subscriber_callback, this, std::placeholders::_1));
    }

    void tailocNode::subscriber_callback(const sensor_msgs::msg::LaserScan msg){
        static std::vector<ndt_cpp::point2> before_points;

        std::vector<ndt_cpp::point2> sensor_points;
        for(size_t i=0; i < msg.ranges.size(); ++i) {
            ndt_cpp::point2 lp;
            double th = msg.angle_min + msg.angle_increment * i;
            double r = msg.ranges[i];
            if (msg.range_min < r && r < msg.range_max) {
                lp.x = r * cos(th); lp.y = r * sin(th);
                sensor_points.push_back(lp);
            }
        }

        if(before_points.size() < 10){
            for(const auto& point : sensor_points){
                before_points.push_back(point);
            }
            return;
        }

        size_t minSize = std::min(sensor_points.size(), before_points.size());
        sensor_points.resize(minSize);
        before_points.resize(minSize);

        ndt_cpp::mat3x3 trans_mat;
        const auto covs = ndt_cpp::compute_ndt_points(before_points);
        ndt_cpp::ndt_scan_matching(trans_mat, sensor_points, before_points, covs);

        RCLCPP_INFO(get_logger(), "%f,%f", trans_mat.c,trans_mat.f);

        before_points.clear();
        for(const auto& point : sensor_points){
            before_points.push_back(point);
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tailoc_ros2::tailocNode)