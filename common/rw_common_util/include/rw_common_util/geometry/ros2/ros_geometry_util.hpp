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

#ifndef RW_COMMON_UTIL__ROS_GEOMETRY_UTILL_HPP_
#define RW_COMMON_UTIL__ROS_GEOMETRY_UTILL_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <concepts>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rw_common_util/geometry/quaternion/quaternion_util.hpp>
#include <tuple>

namespace rw_util
{
namespace geometry
{

template <std::floating_point T>
geometry_msgs::msg::Quaternion euler_to_rosquat(const T & x, const T & y, const T & z);
geometry_msgs::msg::Quaternion euler_to_rosquat(const geometry_msgs::msg::Vector3 & input);

auto quat_to_euler(const geometry_msgs::msg::Quaternion & input)
  -> std::tuple<double, double, double>;

}  // namespace geometry
}  // namespace rw_util

#endif
