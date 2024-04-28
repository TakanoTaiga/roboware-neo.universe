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
        : rclcpp::Node("wp_to_wp_planner_node", node_option), PathPlanning(), PlanningUtil()
    {   
        sub_current_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "input/pose/current", 0, std::bind(&WP2WPPlannerNode::current_pose_subscriber_callback, this, std::placeholders::_1));
        sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "input/pose/goal", 0, std::bind(&WP2WPPlannerNode::goal_pose_subscriber_callback, this, std::placeholders::_1));

        pub_global_plan_path_ = create_publisher<nav_msgs::msg::Path>(
            "output/global_plan_path", 0);
        pub_debug_area_ = create_publisher<visualization_msgs::msg::Marker>(
            "debug/area",0);
        pub_debug_robot_ = create_publisher<visualization_msgs::msg::Marker>(
            "debug/robot",0);

        RCLCPP_INFO_STREAM(get_logger(),  "Initialize Task Done");
    }

    void WP2WPPlannerNode::current_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& msg){
        current_pose = msg;
        
        boost_type::polygon_2d_lf map;
        boost::geometry::exterior_ring(map) = 
            boost::assign::list_of<boost_type::point_2d_lf>(-0.150,0.150)(-5.150,0.150)(-5.150,3.150)(-0.150,3.150)(-0.150,0.150);

        boost_type::polygon_2d_lf robot;
        boost::geometry::exterior_ring(robot) = 
            boost::assign::list_of<boost_type::point_2d_lf>(-0.5,-0.5)(-0.5,0.5)(0.5,0.5)(0.5,-0.5)(-0.5,-0.5);

        const auto result = check_pose_in_map(
            msg,
            map,
            robot
        );
        auto robot_msg = polygon_to_ros("base_link", get_clock()->now(), robot, 0);
        if(result == outside_pose){
            robot_msg.color.g = 0.0;
            robot_msg.color.r = 1.0;
        }

        pub_debug_area_->publish(polygon_to_ros("map", get_clock()->now(), map, 0));
        pub_debug_robot_->publish(robot_msg);

    }

    void WP2WPPlannerNode::goal_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& msg){
        boost_type::polygon_2d_lf map;
        boost::geometry::exterior_ring(map) = 
            boost::assign::list_of<boost_type::point_2d_lf>(-0.150,0.150)(-5.150,0.150)(-5.150,3.150)(-0.150,3.150)(-0.150,0.150);

        boost_type::polygon_2d_lf robot;
        boost::geometry::exterior_ring(robot) = 
            boost::assign::list_of<boost_type::point_2d_lf>(-0.5,-0.5)(-0.5,0.5)(0.5,0.5)(0.5,-0.5)(-0.5,-0.5);

        auto nav_path = nav_msgs::msg::Path();
        nav_path.header.frame_id = "map";
        nav_path.header.stamp = get_clock()->now();

        const auto result = global_path_init(
            current_pose,
            msg,
            map,
            robot,
            nav_path,
            get_logger()
        );

        pub_global_plan_path_->publish(nav_path);
    }


}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wp2wp_planner::WP2WPPlannerNode)