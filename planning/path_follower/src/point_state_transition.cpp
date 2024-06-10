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

#include "path_follower/point_state_transition.hpp"

namespace point_state_transion
{
    state_manager::state_manager()
    {
        current_pose_ready = false;
        path_ready = false;

        current_pose = geometry_msgs::msg::PoseStamped();
    }

    void state_manager::paramset(const double& pos_err_max, const double& angle_err_max)
    {
        param_pos_err_max = pos_err_max;
        param_angle_err_max = angle_err_max;
    }

    void state_manager::setpath(const nav_msgs::msg::Path& path)
    {
        if(path.poses.size() == 0){ return; }
        current_path = path;
        path_ready = true;
    }

    void state_manager::setCurrentPose(const geometry_msgs::msg::PoseStamped& pose)
    {
        current_pose = pose;
        current_pose_ready = true;
    }

    bool state_manager::isReady()
    {
        return path_ready && current_pose_ready;
    }

    void state_manager::setWait()
    {
        path_ready = false;
        current_pose_ready = false;
        current_path = nav_msgs::msg::Path();
    }

    bool state_manager::comp(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2)
    {
        return norm2(p1.pose.position, p2.pose.position) < param_pos_err_max &&
               norm2(p1.pose.orientation, p2.pose.orientation) < param_angle_err_max;
    }

    double state_manager::norm2(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        using rw_common_util::geometry::operator-;
        const auto dp = p1 - p2;
        return std::hypot(dp.x, dp.y);
    }

    double state_manager::norm2(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2)
    {
        const auto q1_euler = rw_common_util::geometry::quat_to_euler(q1);
        const auto q2_euler = rw_common_util::geometry::quat_to_euler(q2);

        auto yaw_diff = q1_euler.yaw - q2_euler.yaw;
        if (yaw_diff > M_PI) {
            yaw_diff -= 2 * M_PI;
        } else if (yaw_diff < -M_PI) {
            yaw_diff += 2 * M_PI;
        }
        return yaw_diff;
    }
}