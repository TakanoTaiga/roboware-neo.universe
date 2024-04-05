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

#ifndef TAILOC_NODE_HPP_
#define TAILOC_NODE_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "tailoc_ros2/ndt_cpp.hpp"

namespace tailoc_ros2
{
    class tailocNode : public rclcpp::Node
    {
    public:
        explicit tailocNode(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
        void subscriber_callback(const sensor_msgs::msg::LaserScan msg);
    };
} // namespace tailoc_ros2

#endif