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

#ifndef SINMPLE_PLANNING_SIMULATOR_NODE_HPP_
#define SINMPLE_PLANNING_SIMULATOR_NODE_HPP_

#include <vector>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace simple_planning_simulator
{
    class SimplePlanningSimulatorNode : public rclcpp::Node
    {
    public:
        explicit SimplePlanningSimulatorNode(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr pub_tf_timer_;

        geometry_msgs::msg::Twist twist_msg;
        geometry_msgs::msg::TransformStamped tf_stamp;
        double roll;

        void timer_callback();
        void subscriber_callback(const geometry_msgs::msg::Twist& msg);

    };
} // namespace simple_planning_simulator

#endif