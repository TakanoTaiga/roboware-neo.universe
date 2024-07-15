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

#ifndef PATH_FOLLOWER_NODE_HPP_
#define PATH_FOLLOWER_NODE_HPP_

#include <memory>
#include <fstream>
#include <vector>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rw_planning_msg/msg/action_result.hpp>
#include <rw_planning_msg/msg/task_action.hpp>
#include <rw_common_util/geometry.hpp>

#include "path_follower/point_state_transition.hpp"

namespace path_follower
{
    enum planner_status
    {
        ready,
        not_found
    };

    class PathFollowerNode : public rclcpp::Node
    {
    public:
        explicit PathFollowerNode(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_current_pose_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_nav_path_;
        rclcpp::Subscription<rw_planning_msg::msg::TaskAction>::SharedPtr sub_task_action_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
        rclcpp::Publisher<rw_planning_msg::msg::ActionResult>::SharedPtr pub_action_result;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_cmd_pose;

        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_debug_current_angle;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_debug_control_angle;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_debug_target_pose;

        rclcpp::TimerBase::SharedPtr control_timer_;

        int32_t task_id;

        point_state_transion::state_manager point_state_manager;

        void timer_callback();
        void current_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& sub_msg_pose);
        void nav_path_subscriber_callback(const nav_msgs::msg::Path& sub_msg_path);
        void task_action_subscriber_callback(const rw_planning_msg::msg::TaskAction& action_msg);

        double position_tolerance;
        double angle_tolerance;
    };
} // namespace path_follower

#endif