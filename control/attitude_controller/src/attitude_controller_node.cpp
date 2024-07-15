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

#include "attitude_controller/attitude_controller_node.hpp"

namespace attitude_controller
{
    attitude_controller_node::attitude_controller_node(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("attitude_controller_node", node_option)
    {   
        pub_twist_ = create_publisher<geometry_msgs::msg::Twist>(
            "output/twist", 0);
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "input/sensor_pose", 0, std::bind(&attitude_controller_node::pose_subscriber_callback, this, std::placeholders::_1));
        sub_cmd_pose_ = create_subscription<geometry_msgs::msg::Pose>(
            "input/cmd_pose", 0, std::bind(&attitude_controller_node::cmd_pose_subscriber_callback, this, std::placeholders::_1));
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&attitude_controller_node::timer_callback, this));
        
        angle_p_gain = declare_parameter<double>("p_gain" , 1.0);
    }

    void attitude_controller_node::pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& msg)
    {
        sensor_pose = msg.pose;
    }

    void attitude_controller_node::cmd_pose_subscriber_callback(const geometry_msgs::msg::Pose& msg)
    {
        cmd_pose = msg;
    }

    void attitude_controller_node::timer_callback()
    {
        const auto sensor_z = rw_common_util::geometry::quat_to_euler(sensor_pose.orientation).yaw;
        const auto cmd_z = rw_common_util::geometry::quat_to_euler(cmd_pose.orientation).yaw;

        auto normalize_angle = [](double angle){
            while (angle > M_PI)
                angle -= 2.0 * M_PI;
            while (angle < -M_PI)
                angle += 2.0 * M_PI;
            return angle;
        };

        const auto normalized_sense_z = normalize_angle(sensor_z);
        const auto normalized_cmd_z = normalize_angle(cmd_z);

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x =  cmd_pose.position.x * cos(normalized_sense_z) + cmd_pose.position.y * sin(normalized_sense_z);
        twist.linear.y = -cmd_pose.position.x * sin(normalized_sense_z) + cmd_pose.position.y * cos(normalized_sense_z);
        twist.angular.z = angle_p_gain * (normalize_angle(normalized_sense_z - normalized_cmd_z));
        pub_twist_->publish(twist);

    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(attitude_controller::attitude_controller_node)
