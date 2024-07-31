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

#ifndef ATTITUDE_CONTROLLER_NODE_
#define ATTITUDE_CONTROLLER_NODE_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rw_common_util/geometry.hpp>

namespace attitude_controller
{

    class attitude_controller_node : public rclcpp::Node
    {
    public:
        explicit attitude_controller_node(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_cmd_pose_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        void pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& msg);
        void cmd_pose_subscriber_callback(const geometry_msgs::msg::Pose& msg);
        void timer_callback();
        void power_saver(geometry_msgs::msg::Twist& cmd_vel);

        geometry_msgs::msg::Pose sensor_pose;
        geometry_msgs::msg::Pose cmd_pose;

        double angle_p_gain;
        double max_linear_velocity;
        double max_angular_velocity;

    };
} // namespace attitude_controller

#endif