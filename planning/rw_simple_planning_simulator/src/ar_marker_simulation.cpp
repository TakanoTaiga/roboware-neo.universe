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

#include "rw_simple_planning_simulator/ar_marker_simulation.hpp"

namespace rw_simple_planning_simulator
{
    visualization_msgs::msg::Marker polygon_to_ros(
        std::string frame_id,
        builtin_interfaces::msg::Time stamp,
        polygon_2d& poly,
        int32_t id
    ){
        auto marker_msg = visualization_msgs::msg::Marker();
        marker_msg.header.stamp = stamp;
        marker_msg.header.frame_id = frame_id;
        marker_msg.ns = "polygon";
        marker_msg.id = id;
        marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.scale.x = 0.05;
        marker_msg.color.r = 1.0;
        marker_msg.color.g = 0.0;
        marker_msg.color.b = 0.0;
        marker_msg.color.a = 0.8;

        for(const auto& point : poly.outer()){
            auto ros_point = geometry_msgs::msg::Point();
            ros_point.x = point.x();
            ros_point.y = point.y();
            marker_msg.points.push_back(ros_point);
        }

        return marker_msg;
    }

    ar_marker_simulation::ar_marker_simulation(
        double camera_fov,
        double search_range_max,
        double search_range_min
    ){
        const double fov_half = camera_fov / 2.0;
        const double fov_res = camera_fov / 10.0;

        std::vector<std::pair<double, double>> unit_angle_vec;
        unit_angle_vec.reserve(10);
        for(double iter = -1.0 * fov_half; iter <= fov_half; iter += fov_res)
        {
            unit_angle_vec.emplace_back(
                std::cos(iter), std::sin(iter)
            );
        }

        for(const auto& [x, y] : unit_angle_vec)
        {
            boost::geometry::append(base_detection_area.outer(), point_2d(x * search_range_min, y * search_range_min));
        }

        for(auto it = unit_angle_vec.rbegin(); it != unit_angle_vec.rend(); ++it)
        {
            const auto& [x, y] = *it;
            boost::geometry::append(base_detection_area.outer(), point_2d(x * search_range_max, y * search_range_max));
        }

        const auto& [x, y] = unit_angle_vec.at(0);
        boost::geometry::append(base_detection_area.outer(), point_2d(x * search_range_min, y * search_range_min));

        boost::geometry::exterior_ring(ar_maker_model) = boost::assign::list_of<point_2d>(-0.05, 0)(0.05, 0);

        start_time = std::chrono::high_resolution_clock::now();
    }

    void ar_marker_simulation::set_param(const std::string& path, const int& id)
    {
        std::ifstream ifs_csv_file(path);
        std::string line;

        while (std::getline(ifs_csv_file, line)) {
            std::istringstream ss(line);
            std::string token;
            double values[4];
            int index = 0;
            while (std::getline(ss, token, ',')) {
                values[index++] = std::stod(token);
            }
            param_data[id].push_back(std::make_pair(values[0], std::make_tuple(values[1], values[2],  values[3])));
        }
    }

    visualization_msgs::msg::MarkerArray ar_marker_simulation::pub_pose(
            std::unique_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster, 
            const builtin_interfaces::msg::Time& time
    ){
        const auto end_time = std::chrono::high_resolution_clock::now();
        auto sec = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()) / 1000;

        auto arry_msg = visualization_msgs::msg::MarkerArray();

        for(const auto& [id, data] : param_data)
        {
            static double x;
            static double y;
            static double angle;
            for(const auto& [time, pos]: data)
            {
                if(time < sec){continue;}
                const auto& [x_, y_, angle_] = pos;
                x = x_;
                y = y_;
                angle = angle_;
                break;
            }

            polygon_2d ar_maker_map_model;
            boost::geometry::transform(ar_maker_model, ar_maker_map_model, r_tf(angle));
            polygon_2d result;
            boost::geometry::transform(ar_maker_map_model, result, t_tf(x, y));
            ar_maker_map_model = result;

            auto message = polygon_to_ros("map", time, ar_maker_map_model, 0);
            if(boost::geometry::within(ar_maker_map_model, detection_area))
            {
                message.color.r = 0.0;
                message.color.g = 1.0;
                message.color.b = 0.0;
            }
            else
            {
                message.color.r = 1.0;
                message.color.g = 0.0;
                message.color.b = 0.0;
                arry_msg.markers.push_back(message);
                continue;
            }

            geometry_msgs::msg::TransformStamped transformStamped;

            transformStamped.header.stamp = time;
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = "marker_" + std::to_string(id);
            transformStamped.transform.translation.x = x;
            transformStamped.transform.translation.y = y;
            transformStamped.transform.translation.z =  0.5;
            transformStamped.transform.rotation = rw_common_util::geometry::euler_to_rosquat(0.0, 0.0, angle);
            tf_broadcaster->sendTransform(transformStamped);
            arry_msg.markers.push_back(message);
        }
        return arry_msg;
    }


    void ar_marker_simulation::set_current_pose(const geometry_msgs::msg::Pose& pose)
    {
        const auto rpy_pose = rw_common_util::geometry::quat_to_euler(pose.orientation);
        boost::geometry::transform(base_detection_area, detection_area, r_tf(M_PI - rpy_pose.yaw));
        polygon_2d result;
        boost::geometry::transform(detection_area, result, t_tf(pose.position.x, pose.position.y));
        detection_area = result;
    }
}