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

#include "rw_common_util/geometry.hpp"

namespace rw_common_util
{
    namespace geometry
    {
        double normalize_angle(const double& input)
        {
            auto angle = input;
            while (angle > M_PI)
                angle -= 2.0 * M_PI;
            while (angle < -M_PI)
                angle += 2.0 * M_PI;
            return angle;
        }

        double hypot(const geometry_msgs::msg::Vector3& vec)
        {
            return std::hypot(vec.x, vec.y, vec.z);
        }

        double hypot(const geometry_msgs::msg::Point& point)
        {
            return std::hypot(point.x, point.y, point.z);
        }

        geometry_msgs::msg::Vector3 make_vec3(const double& x, const double& y, const double& z)
        {
            geometry_msgs::msg::Vector3 v;
            v.x = x;
            v.y = y;
            v.z = z;
            return v;
        }

        geometry_msgs::msg::Vector3 make_vec3(const geometry_msgs::msg::Point& point)
        {
            return make_vec3(point.x, point.y, point.z);
        }

        geometry_msgs::msg::Point make_point(const double& x, const double& y, const double& z)
        {
            geometry_msgs::msg::Point v;
            v.x = x;
            v.y = y;
            v.z = z;
            return v;
        }

        geometry_msgs::msg::Point make_point(const geometry_msgs::msg::Vector3& vec)
        {
            return make_point(vec.x, vec.y, vec.z);
        }

        template <typename T>
        rw_quat<T> euler_to_quat(const T& roll, const T& pitch, const T& yaw)
        {
            tf2::Quaternion q;
            q.setEuler(roll, pitch, yaw);
            return { q.x(), q.y(), q.z(), q.w() };
        }

        template <typename T>
        rw_quat<T> euler_to_quat(const rw_euler<T>& input)
        {
            return euler_to_quat(input.roll, input.pitch, input.yaw);
        }

        geometry_msgs::msg::Quaternion euler_to_rosquat(
            const double& roll, const double& pitch, const double& yaw)
        {
            tf2::Quaternion q;
            q.setEuler(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion result;
            result.x = q.x();
            result.y = q.y();
            result.z = q.z();
            result.w = q.w();
            return result;        
        }

        geometry_msgs::msg::Quaternion euler_to_rosquat(
            const rw_euler<double>& input)
        {
            return euler_to_rosquat(input.roll, input.pitch, input.yaw);
        }

        template <typename T>
        rw_euler<T> quat_to_euler(const T& x, const T& y, const T& z, const T& w)
        {
            tf2::Quaternion quat_pose;
            quat_pose.setValue(x,y,z,w);
            tf2::Matrix3x3 mat_pose(quat_pose);
            rw_euler<T> result;
            mat_pose.getRPY(result.roll, result.pitch, result.yaw);
            return result;
        }

        template <typename T>
        rw_euler<T> quat_to_euler(const rw_quat<T>& input)
        {
            return quat_to_euler(input.x, input.y, input.z, input.w);
        }
                
        rw_euler<double> quat_to_euler(const geometry_msgs::msg::Quaternion& input)
        {
            return quat_to_euler(input.x, input.y, input.z, input.w);
        }
    }
}
