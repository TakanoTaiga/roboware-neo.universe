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

#ifndef PATH_PLANNING_HPP_
#define PATH_PLANNING_HPP_

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rw_common_util/geometry.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "wp2wp_planner/planning_util.hpp"

namespace wp2wp_planner
{
    enum status
    {
        outside_pose,
        non_error
    };

    class PathPlanning
    {
    public:
        explicit PathPlanning();

        status global_path_init(
            const geometry_msgs::msg::PoseStamped& pose_current,
            const geometry_msgs::msg::PoseStamped& pose_goal,
            const boost_type::polygon_2d_lf& map,
            const boost_type::polygon_2d_lf& robot,
            nav_msgs::msg::Path& result_path,
            rclcpp::Logger logger
        );

        status check_pose_in_map(
            const geometry_msgs::msg::PoseStamped& pose,
            const boost_type::polygon_2d_lf& map,
            const boost_type::polygon_2d_lf& robot
        );

        void init_path_generator(
            const geometry_msgs::msg::PoseStamped& pose_current,
            const geometry_msgs::msg::PoseStamped& pose_goal,
            nav_msgs::msg::Path& result_path
        );

        status check_path_in_map(
            const nav_msgs::msg::Path& result_path,
            const boost_type::polygon_2d_lf& map,
            const boost_type::polygon_2d_lf& robot
        );

        status map_avoidance_planner(
            nav_msgs::msg::Path& move_path,
            const boost_type::polygon_2d_lf& map,
            const boost_type::polygon_2d_lf& robot
        );

    private:
    };
}

#endif