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

#include "wp2wp_planner/WP2WPPlannerNode.hpp"

geometry_msgs::msg::PoseStamped 
findPointC(
    const geometry_msgs::msg::PoseStamped& pose_origin,
    const geometry_msgs::msg::PoseStamped& pose_goal,
    const double& l
){
    const double ab_x = pose_goal.pose.position.x - pose_origin.pose.position.x;
    const double ab_y = pose_goal.pose.position.y - pose_origin.pose.position.y;

    const double lengthAB = std::sqrt(ab_x * ab_x + ab_y * ab_y);

    const double unit_x = ab_x / lengthAB;
    const double unit_y = ab_y / lengthAB;

    const double c_x = pose_origin.pose.position.x + unit_x * l;
    const double c_y = pose_origin.pose.position.y + unit_y * l;

    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.frame_id = "map";
    pose.header.stamp = pose_goal.header.stamp;
    pose.pose.position.x = c_x;
    pose.pose.position.y = c_y;
    return pose;
}

namespace wp2wp_planner
{
    WP2WPPlannerNode::WP2WPPlannerNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("wp_to_wp_planner_node", node_option)
    {   
        sub_current_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/initialpose", 0, std::bind(&WP2WPPlannerNode::current_pose_subscriber_callback, this, std::placeholders::_1));
        sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 0, std::bind(&WP2WPPlannerNode::goal_pose_subscriber_callback, this, std::placeholders::_1));

        pub_global_plan_path_ = create_publisher<nav_msgs::msg::Path>(
            "output/global_plan_path", 0);

        RCLCPP_INFO_STREAM(get_logger(),  "Initialize Task Done");
    }

    void WP2WPPlannerNode::current_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& msg){
        current_pose = msg;
        RCLCPP_INFO_STREAM(get_logger(),  "sub");
    }

    void WP2WPPlannerNode::goal_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& msg){
        const double ab_x = msg.pose.position.x - current_pose.pose.position.x;
        const double ab_y = msg.pose.position.y - current_pose.pose.position.y;

        const double length = std::sqrt(ab_x * ab_x + ab_y * ab_y);

        auto nav_path = nav_msgs::msg::Path();
        for(double i = 0; i < length; i += length / 10){
            auto pose = findPointC(current_pose, msg, i);
            nav_path.poses.push_back(pose);
        }
        nav_path.header.frame_id = "map";
        nav_path.header.stamp = get_clock()->now();

        pub_global_plan_path_->publish(nav_path);
        RCLCPP_INFO_STREAM(get_logger(),  "Pub");
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wp2wp_planner::WP2WPPlannerNode)