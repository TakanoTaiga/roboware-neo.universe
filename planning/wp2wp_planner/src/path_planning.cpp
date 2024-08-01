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
namespace path_planning
{
    status global_path_init(
        const geometry_msgs::msg::PoseStamped& pose_current,
        const geometry_msgs::msg::PoseStamped& pose_goal,
        const boost_type::polygon_2d_lf& map,
        const boost_type::polygon_2d_lf& robot,
        const double& v_max,
        nav_msgs::msg::Path& result_path,
        rclcpp::Logger logger
    ){
        if(check_pose_in_map(pose_current, map, robot) == status::outside_pose){
            RCLCPP_ERROR_STREAM(logger,  
                "pose_current is outside map. " << 
                pose_current.pose.position.y << 
                " , " << 
                pose_current.pose.position.y);
            return status::outside_pose;
        }

        if(check_pose_in_map(pose_goal, map, robot) == status::outside_pose){
            RCLCPP_ERROR_STREAM(logger,  
                "pose_goal is outside map. " << 
                pose_goal.pose.position.y << 
                " , " << 
                pose_goal.pose.position.y);
            return status::outside_pose;
        }

        init_path_generator(
            pose_current,
            pose_goal,
            v_max,
            result_path
        );

        if(check_path_in_map(result_path, map, robot) == status::outside_pose){
            map_avoidance_planner(result_path, map, robot);
        }

            if(check_path_in_map(result_path, map, robot) == status::outside_pose){
            RCLCPP_ERROR_STREAM(logger,  "path is outside map");
            return status::outside_pose;
        }

        return status::non_error;
    }

    status check_pose_in_map(
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
            return status::non_error;
        }else{
            return status::outside_pose;
        }
    }

    void init_path_generator(
        const geometry_msgs::msg::PoseStamped& pose_current,
        const geometry_msgs::msg::PoseStamped& pose_goal,
        const double& v_max,
        nav_msgs::msg::Path& result_path
    ) {
        const double ab_x = pose_goal.pose.position.x - pose_current.pose.position.x;
        const double ab_y = pose_goal.pose.position.y - pose_current.pose.position.y;
        const double length = std::sqrt(ab_x * ab_x + ab_y * ab_y);

        const auto current_rpy = rw_common_util::geometry::quat_to_euler(pose_current.pose.orientation);
        const auto goal_rpy = rw_common_util::geometry::quat_to_euler(pose_goal.pose.orientation);

        auto sigmoid = [](double x) {
            return 1.0 / (1.0 + std::exp(-x));
        };

        for(double l = 0.0; l < length; l += 0.05) {
            const double unit_x = ab_x / length;
            const double unit_y = ab_y / length;
            const double c_x = pose_current.pose.position.x + unit_x * l;
            const double c_y = pose_current.pose.position.y + unit_y * l;

            double yaw_diff = goal_rpy.yaw - current_rpy.yaw;
            if (yaw_diff > M_PI) {
                yaw_diff -= 2 * M_PI;
            } else if (yaw_diff < -M_PI) {
                yaw_diff += 2 * M_PI;
            }

            double progress = l / length;
            double smooth_progress = sigmoid(8.0 * (progress - 0.5));
            const double interp_yaw = current_rpy.yaw + yaw_diff * smooth_progress;

            double interp_velocity;
            const auto acc_length = 0.1 * length;

            if (progress < 0.1) {
                interp_velocity = v_max * (progress / 0.1);
            } else if (progress > 0.9) {
                interp_velocity = v_max * ((1.0 - progress) / 0.1);
            } else {
                interp_velocity = v_max;
            }

            auto pose = geometry_msgs::msg::PoseStamped();
            pose.header = result_path.header;
            pose.pose.position.x = c_x;
            pose.pose.position.y = c_y;
            pose.pose.position.z = interp_velocity;
            pose.pose.orientation = rw_common_util::geometry::euler_to_rosquat(0.0, 0.0, interp_yaw);

            result_path.poses.push_back(pose);
        }
        auto final_pose = geometry_msgs::msg::PoseStamped();
        final_pose.header = result_path.header;
        final_pose.pose = pose_goal.pose;
        result_path.poses.push_back(final_pose);
    }

    status check_path_in_map(
        const nav_msgs::msg::Path& result_path,
        const boost_type::polygon_2d_lf& map,
        const boost_type::polygon_2d_lf& robot
    ){
        for(const auto& point : result_path.poses){
            const auto result = check_pose_in_map(point, map, robot);
            if(result == status::outside_pose){
                return status::outside_pose;
            }
        }
        return status::non_error;
    }

    status map_avoidance_planner(
        nav_msgs::msg::Path& move_path,
        const boost_type::polygon_2d_lf& map,
        const boost_type::polygon_2d_lf& robot
    ){
        std::vector<std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>> error_poses;
        for (size_t i = 0; i < move_path.poses.size(); ++i) {
            const auto& point = move_path.poses[i];
            if (check_pose_in_map(point, map, robot) == status::non_error) continue;

            geometry_msgs::msg::PoseStamped next_point;
            if (i + 1 < move_path.poses.size()) {
                next_point = move_path.poses[i + 1];
            } else if (i > 0) {
                next_point = move_path.poses[i - 1];
            }

            const auto vec_x = point.pose.position.x - next_point.pose.position.x;
            const auto vec_y = point.pose.position.y - next_point.pose.position.y;
            const auto length = std::hypot(vec_x, vec_y);
            const auto perp_vec_x = vec_y / length;
            const auto perp_vec_y = -vec_x / length;

            for(double sc = 0.0; sc < 2.0; sc += 0.01)
            {
                auto point_ = point;
                point_.pose.position.x += perp_vec_x * sc;
                point_.pose.position.y += perp_vec_y * sc;
                if(check_pose_in_map(point_, map, robot) == status::non_error){
                    move_path.poses[i] = point_;
                    break;
                }

                point_.pose.position.x -= 2.0 * perp_vec_x * sc;
                point_.pose.position.y -= 2.0 * perp_vec_y * sc;
                if(check_pose_in_map(point_, map, robot) == status::non_error){
                    move_path.poses[i] = point_;
                    break;
                }
            }
        }
        return status::non_error;
    }
}
}