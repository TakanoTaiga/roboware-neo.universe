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

#ifndef OTOS_DRIVER_NODE_HPP_
#define OTOS_DRIVER_NODE_HPP_

#include <vector>
#include <memory>
#include <chrono>
#include <sstream>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rw_common_util/geometry.hpp>

namespace otos_driver
{
    class otos_driver_node : public rclcpp::Node
    {
    public:
        explicit otos_driver_node(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sensor_data;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_current_pose_;

        void subscriber_callback(const std_msgs::msg::String& msg);
    };
} // namespace otos_driver

#endif