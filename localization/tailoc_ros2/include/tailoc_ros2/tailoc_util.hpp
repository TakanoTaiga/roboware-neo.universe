// Copyright 2023 Hakoroboken
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

#ifndef TAILOC_UTIL_HPP_
#define TAILOC_UTIL_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Quaternion.h>


#include "tailoc_ros2/ndt_cpp.hpp"

namespace tailoc_util
{
    void laserscan_to_ndtcpp_point2(
        const sensor_msgs::msg::LaserScan& laser_scan,
        std::vector<ndt_cpp::point2>& ndtcpp_point2
    );

    geometry_msgs::msg::TransformStamped 
        point3_to_tf_stamp(
            const ndt_cpp::point3& odom,
            const builtin_interfaces::msg::Time& stamp
    );

    geometry_msgs::msg::PoseStamped
        point3_to_pose_stamp(
            const ndt_cpp::point3& odom,
            const builtin_interfaces::msg::Time& stamp
        );
}

#endif