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

#ifndef JOY_TO_TWIST_NODE_HPP_
#define JOY_TO_TWIST_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace joy_to_twist
{

    class JoyToTwistNode : public rclcpp::Node
    {
    public:
        explicit JoyToTwistNode(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

        void subscriber_callback(const sensor_msgs::msg::Joy& msg);

    };
} // namespace joy_to_twist

#endif