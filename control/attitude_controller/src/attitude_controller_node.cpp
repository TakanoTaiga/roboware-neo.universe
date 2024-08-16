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
        pub_debug_pid_ = create_publisher<rw_common_msgs::msg::PIDDebug>(
            "debug/pid_debug", 0);
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "input/sensor_pose", 0, std::bind(&attitude_controller_node::pose_subscriber_callback, this, std::placeholders::_1));
        sub_cmd_pose_ = create_subscription<geometry_msgs::msg::Pose>(
            "input/cmd_pose", 0, std::bind(&attitude_controller_node::cmd_pose_subscriber_callback, this, std::placeholders::_1));
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&attitude_controller_node::timer_callback, this));
        
        angle_p_gain = declare_parameter<double>("p_gain" , 1.0);
        angle_d_gain = declare_parameter<double>("d_gain" , 0.1);
        max_linear_velocity = declare_parameter<double>("max_velocity_m/s" , 3.0);
        max_angular_velocity = declare_parameter<double>("max_angular_m/s" , 1.0);

        previous_time = this->now();
        previous_sensor_z = 0.0;
        
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

        const auto normalized_sense_z = rw_common_util::geometry::normalize_angle(sensor_z);
        const auto normalized_cmd_z = rw_common_util::geometry::normalize_angle(cmd_z);

        // Angle PD Cotnrol 
        const auto error = rw_common_util::geometry::normalize_angle(normalized_sense_z - normalized_cmd_z);
        const auto p_term = angle_p_gain * error;

        const auto current_time = this->now();
        const auto dt = (current_time - previous_time).seconds();
        previous_time = current_time;
        const auto sensor_z_dot = (normalized_sense_z - previous_sensor_z) / dt;
        previous_sensor_z = normalized_sense_z;
        const auto d_term = angle_d_gain * sensor_z_dot;

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x =  cmd_pose.position.x * cos(normalized_sense_z) + cmd_pose.position.y * sin(normalized_sense_z);
        twist.linear.y = -cmd_pose.position.x * sin(normalized_sense_z) + cmd_pose.position.y * cos(normalized_sense_z);
        twist.angular.z = p_term - d_term;

        auto pid_debug_msg = rw_common_msgs::msg::PIDDebug();
        pid_debug_msg.order = normalized_cmd_z;
        pid_debug_msg.sensor = normalized_sense_z;
        pid_debug_msg.control = twist.angular.z;
        pid_debug_msg.p = p_term;
        pid_debug_msg.d = d_term;
        pub_debug_pid_->publish(pid_debug_msg);

        power_saver(twist);
        pub_twist_->publish(twist);

    }

    void attitude_controller_node::power_saver(geometry_msgs::msg::Twist& cmd_vel)
    {
        auto pow2 = [](double input){
            return input * input;
        };

        auto linear_velocity_saver = [pow2](geometry_msgs::msg::Vector3 input, double max_velocity){
            const auto norm = std::hypot(input.x, input.y);
            if(norm < max_velocity){ return input; }
            const auto scaler = std::sqrt(pow2(max_velocity) / (pow2(input.x) + pow2(input.y)));
            auto msg = geometry_msgs::msg::Vector3();
            msg.x = scaler * input.x;
            msg.y = scaler * input.y;
            return msg;
        };

        auto angular_velocity_saver = [pow2](geometry_msgs::msg::Vector3 input, double max_velocity){
            if(std::abs(input.z) < std::abs(max_velocity)){ return input; }
            auto msg = geometry_msgs::msg::Vector3();
            msg.z = std::sqrt(pow2(max_velocity) / pow2(input.z)) * input.z;
            return msg;
        };

        cmd_vel.linear = linear_velocity_saver(cmd_vel.linear, max_linear_velocity);
        cmd_vel.angular = angular_velocity_saver(cmd_vel.angular, max_angular_velocity);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(attitude_controller::attitude_controller_node)
