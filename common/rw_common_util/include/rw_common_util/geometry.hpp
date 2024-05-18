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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>

namespace rw_common_util
{
    namespace geometry
    {   
        template <typename T>
        struct rw_quat
        {
            T x, y, z, w;
        };
        template <typename T>
        struct rw_euler
        {
            T roll, pitch, yaw;
        };

        template <typename T>
        rw_quat<T> euler_to_quat(const T& roll, const T& pitch, const T& yaw);
        template <typename T>
        rw_quat<T> euler_to_quat(const rw_euler<T>& input);

        geometry_msgs::msg::Quaternion euler_to_rosquat(
            const double& roll, const double& pitch, const double& yaw);
        geometry_msgs::msg::Quaternion euler_to_rosquat(
            const rw_euler<double>& input);

        template <typename T>
        rw_euler<T> quat_to_euler(const T& x, const T& y, const T& z, const T& w);

        template <typename T>
        rw_euler<T> quat_to_euler(const rw_quat<T>& input);

        rw_euler<double> quat_to_euler(const geometry_msgs::msg::Quaternion& input);
    }
}