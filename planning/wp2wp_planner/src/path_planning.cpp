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

#include "wp2wp_planner/path_planning.hpp"

namespace wp2wp_planner
{
    PathPlanning::PathPlanning()
    {

    }

    status PathPlanning::global_path_init(
        const geometry_msgs::msg::PoseStamped& pose_current,
        const geometry_msgs::msg::PoseStamped& pose_goal,
        const boost_type::polygon_2d_lf& map,
        const boost_type::polygon_2d_lf& robot,
        nav_msgs::msg::Path& result_path,
        rclcpp::Logger logger
    ){
        if(check_pose_in_map(pose_current, map, robot) == outside_pose){
            RCLCPP_ERROR_STREAM(logger,  "pose_current is outside map");
            return outside_pose;
        }

        if(check_pose_in_map(pose_goal, map, robot) == outside_pose){
            RCLCPP_ERROR_STREAM(logger,  "pose_goal is outside map");
            return outside_pose;
        }

        init_path_generator(
            pose_current,
            pose_goal,
            result_path
        );

        if(check_path_in_map(result_path, map, robot) == outside_pose){
            RCLCPP_ERROR_STREAM(logger,  "path is outside map");
            result_path.poses.clear();
            return path_error;
        }

        return non_error;
    }

    status PathPlanning::check_pose_in_map(
        const geometry_msgs::msg::PoseStamped& pose,
        const boost_type::polygon_2d_lf& map,
        const boost_type::polygon_2d_lf& robot
    ){
        const auto rpy = rw_common_util::geometry::quat_to_euler(pose.pose.orientation);
        
        boost_type::r_tf rotate_translate(rpy.yaw);
        boost_type::polygon_2d_lf pose_rotated_robot;
        boost::geometry::transform(robot, pose_rotated_robot, rotate_translate);

        // transform translate
        boost_type::t_tf transform_translate(pose.pose.position.x, pose.pose.position.y);
        boost_type::polygon_2d_lf pose_transed_robot;
        boost::geometry::transform(pose_rotated_robot, pose_transed_robot, transform_translate);

        // check
        if(boost::geometry::within(pose_transed_robot, map)){
            return non_error;
        }else{
            return outside_pose;
        }
    }

    void PathPlanning::init_path_generator(
        const geometry_msgs::msg::PoseStamped& pose_current,
        const geometry_msgs::msg::PoseStamped& pose_goal,
        nav_msgs::msg::Path& result_path
    ){
        const double ab_x = pose_goal.pose.position.x - pose_current.pose.position.x;
        const double ab_y = pose_goal.pose.position.y - pose_current.pose.position.y;
        const double length = std::sqrt(ab_x * ab_x + ab_y * ab_y);

        const auto current_rpy = rw_common_util::geometry::quat_to_euler(pose_current.pose.orientation);
        const auto goal_rpy = rw_common_util::geometry::quat_to_euler(pose_goal.pose.orientation);

        for(double l = 0.0; l <= length; l+= 0.1){
            const double unit_x = ab_x / length;
            const double unit_y = ab_y / length;
            const double c_x = pose_current.pose.position.x + unit_x * l;
            const double c_y = pose_current.pose.position.y + unit_y * l;

            const double interp_yaw = current_rpy.yaw + (goal_rpy.yaw - current_rpy.yaw) * (l / length);

            auto pose = geometry_msgs::msg::PoseStamped();
            pose.header = result_path.header;
            pose.pose.position.x = c_x;
            pose.pose.position.y = c_y;
            pose.pose.orientation = rw_common_util::geometry::euler_to_rosquat(0.0, 0.0, interp_yaw);

            result_path.poses.push_back(pose);
        }
    }

    status PathPlanning::check_path_in_map(
        const nav_msgs::msg::Path& result_path,
        const boost_type::polygon_2d_lf& map,
        const boost_type::polygon_2d_lf& robot
    ){
        for(const auto& point : result_path.poses){
            const auto result = check_pose_in_map(point, map, robot);
            if(result == outside_pose){
                return outside_pose;
            }
        }
        return non_error;
    }

    status PathPlanning::map_avoidance_planner(
        nav_msgs::msg::Path& move_path,
        const boost_type::polygon_2d_lf& map,
        const boost_type::polygon_2d_lf& robot
    ){
        return non_error;
    }
}