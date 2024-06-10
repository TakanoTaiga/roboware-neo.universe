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

#ifndef POINT_STATE_TRANSITON_HPP_
#define POINT_STATE_TRANSITON_HPP_

#include <utility>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <rw_common_util/geometry.hpp>

namespace point_state_transion
{
    class state_manager 
    {
    public:
        // pos_err_max [m], angle_err_max [rad]
        state_manager();
        void paramset(const double& pos_err_max, const double& angle_err_max);
        void setpath(const nav_msgs::msg::Path& path);
        void setCurrentPose(const geometry_msgs::msg::PoseStamped& pose);
        bool isReady();
        void setWait();
        double norm2(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
        double norm2(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2);

        nav_msgs::msg::Path current_path;
        geometry_msgs::msg::PoseStamped current_pose;

        double param_pos_err_max;
        double param_angle_err_max;

    private:
        bool comp(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2);

        bool path_ready;
        bool current_pose_ready;

    };
}

#endif