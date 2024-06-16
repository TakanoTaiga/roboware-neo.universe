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

#ifndef AR_MARKER_SIMULATION_HPP_
#define AR_MARKER_SIMULATION_HPP_

#include <chrono>
#include <tuple>
#include <fstream>
#include <sstream>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rw_common_util/geometry.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/assign/list_of.hpp>

namespace rw_simple_planning_simulator
{
    using point_2d = boost::geometry::model::d2::point_xy<double>;
    using polygon_2d = boost::geometry::model::polygon<point_2d>;
    using r_tf = boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>;
    using t_tf = boost::geometry::strategy::transform::translate_transformer<double, 2, 2>;

    visualization_msgs::msg::Marker polygon_to_ros(
        std::string frame_id,
        builtin_interfaces::msg::Time stamp,
        polygon_2d& poly,
        int32_t id
    );

    class ar_marker_simulation
    {
    public:
        ar_marker_simulation(
            double camera_fov,
            double search_range_max,
            double search_range_min
        );
        void set_current_pose(const geometry_msgs::msg::Pose& pose);
        visualization_msgs::msg::MarkerArray pub_pose(
            std::unique_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster, 
            const builtin_interfaces::msg::Time& time
        );
        void set_param(const std::string& path, const int& id);

        polygon_2d detection_area; //public debug only
    private:
        polygon_2d base_detection_area;
        polygon_2d ar_maker_model;
        std::__1::chrono::steady_clock::time_point start_time;
        std::map<int, std::vector<std::pair<double, std::tuple<double, double, double>>>> param_data;
    };
} // rw_simple_planning_simulator

#endif // AR_MARKER_SIMULATION_HPP_
