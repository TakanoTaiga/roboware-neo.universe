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

#include "tailoc_ros2/tailoc_util.hpp"

namespace tailoc_util
{
    void laserscan_to_ndtcpp_point2(
        const sensor_msgs::msg::LaserScan& laser_scan,
        std::vector<ndt_cpp::point2>& ndtcpp_point2
    ){
        const auto scan_point_size = laser_scan.ranges.size();
        for(size_t i=0; i < scan_point_size; ++i) {
            ndt_cpp::point2 lp;
            double th = laser_scan.angle_min + laser_scan.angle_increment * i;
            double r = laser_scan.ranges[i];
            if (laser_scan.range_min < r && r < laser_scan.range_max) {
                lp.x = r * cos(th); lp.y = r * sin(th);
                ndtcpp_point2.push_back(lp);
            }
        }
    }

    geometry_msgs::msg::TransformStamped 
        point3_to_tf_stamp(
            const ndt_cpp::point3& odom,
            const builtin_interfaces::msg::Time& stamp
    ){
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        t.transform.translation.x = odom.x;
        t.transform.translation.y = odom.y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, odom.z);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        return t;
    }

    geometry_msgs::msg::PoseStamped
        point3_to_pose_stamp(
            const ndt_cpp::point3& odom,
            const builtin_interfaces::msg::Time& stamp
    ){
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "base_link";

        pose.pose.position.x = odom.x;
        pose.pose.position.y = odom.y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, odom.z);

        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        return pose;
    }

}