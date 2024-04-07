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

#ifndef TAILOC_NODE_HPP_
#define TAILOC_NODE_HPP_

#include <vector>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "tailoc_ros2/ndt_cpp.hpp"

namespace tailoc_ros2
{
    class tailocNode : public rclcpp::Node
    {
    public:
        explicit tailocNode(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_current_pose_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        void subscriber_callback(const sensor_msgs::msg::LaserScan msg);

        ndt_cpp::point3 odom;
        nav_msgs::msg::Path path;
        std::vector<ndt_cpp::point2> map_points;

        ndt_cpp::ndtParam ndt_param;
    };
} // namespace tailoc_ros2

#endif