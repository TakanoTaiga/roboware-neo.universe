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

#include <boost/assign/list_of.hpp>

namespace wp2wp_planner
{
    WP2WPPlannerNode::WP2WPPlannerNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("wp_to_wp_planner_node", node_option)
    {   
        sub_current_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "input/pose/current", 0, std::bind(&WP2WPPlannerNode::current_pose_subscriber_callback, this, std::placeholders::_1));
        sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "input/pose/goal", 0, std::bind(&WP2WPPlannerNode::goal_pose_subscriber_callback, this, std::placeholders::_1));
        sub_task_action_ = create_subscription<rw_planning_msg::msg::TaskAction>(
            "input/task_action", 0, std::bind(&WP2WPPlannerNode::task_action_subscriber_callback, this, std::placeholders::_1));

        pub_global_plan_path_ = create_publisher<nav_msgs::msg::Path>(
            "output/global_plan_path", 0);
        pub_action_result = create_publisher<rw_planning_msg::msg::ActionResult>(
            "output/action_result", 0);

        pub_debug_area_ = create_publisher<visualization_msgs::msg::Marker>(
            "debug/area",0);
        pub_debug_robot_ = create_publisher<visualization_msgs::msg::Marker>(
            "debug/robot",0);

        ply2d_map = polyload::polyload(
            declare_parameter<std::string>("map.plypath" , "kill")
        );
        ply2d_robot = polyload::polyload(
            declare_parameter<std::string>("robot.plypath" , "kill")
        );

        param_max_speed = declare_parameter<double>("max_speed" , 1.0);

        map_pub_timer_  = 
            create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WP2WPPlannerNode::map_pub_timer_callback, this));

        RCLCPP_INFO_STREAM(get_logger(),  "Initialize Task Done");
    }

    void WP2WPPlannerNode::current_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& msg){
        current_pose = msg;

        const auto result = path_planning::check_pose_in_map(
            msg,
            ply2d_map,
            ply2d_robot
        );
        auto robot_msg = planning_util::polygon_to_ros("base_link", get_clock()->now(), ply2d_robot, 0);
        if(result == path_planning::status::outside_pose){
            robot_msg.color.g = 0.0;
            robot_msg.color.r = 1.0;
        }
        pub_debug_robot_->publish(robot_msg);
    }

    void WP2WPPlannerNode::goal_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& msg){
        auto nav_path = nav_msgs::msg::Path();
        nav_path.header.frame_id = "map";
        nav_path.header.stamp = get_clock()->now();

        const auto result = path_planning::global_path_init(
            current_pose,
            msg,
            ply2d_map,
            ply2d_robot,
            param_max_speed,
            nav_path,
            get_logger()
        );

        pub_global_plan_path_->publish(nav_path);
    }

    void WP2WPPlannerNode::task_action_subscriber_callback(const rw_planning_msg::msg::TaskAction& action_msg)
    {
        if(action_msg.task != rw_planning_msg::msg::TaskAction::SETPOSE) return;

        goal_pose_subscriber_callback(action_msg.pose);
    }

    void WP2WPPlannerNode::map_pub_timer_callback()
    {
        pub_debug_area_->publish(planning_util::polygon_to_ros("map", get_clock()->now(), ply2d_map, 0));
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wp2wp_planner::WP2WPPlannerNode)