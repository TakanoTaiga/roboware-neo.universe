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

#include "thrown_object_estimator/thrown_object_estimator_node.hpp"

namespace thrown_object_estimator
{
    thrown_object_estimator_node::thrown_object_estimator_node(const rclcpp::NodeOptions &node_option)
    : rclcpp::Node("thrown_object_estimator_node", node_option)
    {   
       sub_detection_result = create_subscription<rw_common_msgs::msg::TransformArray>(
            "input/object_poses", 0, std::bind(&thrown_object_estimator_node::subscriber_callback, this, std::placeholders::_1));
        pub_ball_path = create_publisher<nav_msgs::msg::Path>(
            "output/ball_path", 0);
    }

    void thrown_object_estimator_node::subscriber_callback(const rw_common_msgs::msg::TransformArray& msg)
    {
        // lambda expressions
        auto convertTimestampToDouble = [](builtin_interfaces::msg::Time time) {
            auto sec = time.sec; 
            auto nanosec = time.nanosec;
            constexpr double NANOSECONDS_IN_SECOND = 1000000000.0;
            return static_cast<double>(sec) + static_cast<double>(nanosec) / NANOSECONDS_IN_SECOND;
        };

        auto minimal_solution_parabola = [](
            const double& time_0, const double& time_1,
            const geometry_msgs::msg::Point& pose_0, const geometry_msgs::msg::Point& pose_1
        ){
            RWCOMMON_UTIL_GEOMETRY_OPERATOR_ALL_IMPORT;
            const auto g = rw_common_util::geometry::make_point(0, 0, 9.8045);
            const auto p0_gravity_comp = pose_0 - g * 0.5 * time_0 * time_0;
            const auto p1_gravity_comp = pose_1 - g * 0.5 * time_1 * time_1;
            const auto v = (p1_gravity_comp - p0_gravity_comp) / (time_1 - time_0);
            const auto p = p0_gravity_comp - v * time_0;
            return std::make_pair(p, v);
        };

        auto compute_inliers = [](
            const std::vector<std::pair<geometry_msgs::msg::Point, double>>& stamp_points,
            const geometry_msgs::msg::Point& p,
            const geometry_msgs::msg::Point& v,
            double inlier_threshold = 0.3
        ){
            RWCOMMON_UTIL_GEOMETRY_OPERATOR_ALL_IMPORT;
            const auto g = rw_common_util::geometry::make_point(0, 0, 9.8045);
            const auto n = stamp_points.size();
            int inliers = 0;
            for(const auto& [point, time] : stamp_points)
            {
                const auto point_gravity_comp = point - g * 0.5 * time * time;
                const auto error = point_gravity_comp - (v * time + p);
                const auto error_hypot = rw_common_util::geometry::hypot(error);
                inliers += error_hypot < inlier_threshold ? 1 : 0;
            }
            return inliers;
        };


        // ransac
        for(const auto& pose: msg.transforms)
        {
            const auto& tf = pose.transform.translation;
            if(tf.z < 0.5){continue;}
            if(tf.z < 1.0){continue;}
            ball_positions_stamped.push_back(
                std::make_pair(
                    rw_common_util::geometry::make_point(tf), 
                    convertTimestampToDouble(pose.header.stamp)
                ));
        }
        if(ball_positions_stamped.empty()){return;}

        std::vector<std::pair<geometry_msgs::msg::Point, double>> bps_time_zero_start;
        const auto& first_time = ball_positions_stamped.at(0).second;

        for(const auto& [point, time] : ball_positions_stamped)
        {
            bps_time_zero_start.push_back(
                std::make_pair(
                    point, 
                    time - first_time
                ));
        }

        int max_num_inliers = 0;
        geometry_msgs::msg::Point best_p, best_v;
        std::mt19937 mt{ std::random_device{}() };
        std::uniform_int_distribution<int> dist(0, bps_time_zero_start.size() - 1);
        for(auto i = 0; i < 100; ++i)
        {
            const auto& [p0, t0] = bps_time_zero_start.at(dist(mt));
            const auto& [p1, t1] = bps_time_zero_start.at(dist(mt));
            const auto [p, v] = minimal_solution_parabola(t0, t1, p0, p1);
            const auto num_inliers = compute_inliers(bps_time_zero_start, p, v);
            if(num_inliers > max_num_inliers)
            {
                max_num_inliers = num_inliers;
                best_p = p;
                best_v = v;
            }
            auto estimated_outlier_prob = 1.0 - static_cast<double>(num_inliers) / bps_time_zero_start.size();
            estimated_outlier_prob = std::max(0.000001, std::min(0.99999, estimated_outlier_prob));
        }

        nav_msgs::msg::Path path;
        path.header.frame_id = "base_link";
        path.header.stamp = get_clock()->now();
        for(double t = -1.0; t < 1.0; t += 0.05)
        {
            RWCOMMON_UTIL_GEOMETRY_OPERATOR_ALL_IMPORT;
            const auto g = rw_common_util::geometry::make_point(0, 0, 9.8045);
            geometry_msgs::msg::PoseStamped msg;
            msg.header = path.header;
            msg.pose.position = best_v * t + best_p - g * 0.5 * t * t;
            if(msg.pose.position.z < 0.0){continue;}
            path.poses.push_back(msg);
        }
        pub_ball_path->publish(path);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(thrown_object_estimator::thrown_object_estimator_node)
