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

#include "rw_common_util/geometry/ros2/ros_geometry_util.hpp"

namespace rw_util
{
namespace geometry
{

template <std::floating_point T>
geometry_msgs::msg::Quaternion euler_to_rosquat(const T & x, const T & y, const T & z)
{
  tf2::Quaternion q;
  q.setEuler(x, y, z);
  geometry_msgs::msg::Quaternion result;
  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  result.w = q.w();
  return result;
}

geometry_msgs::msg::Quaternion euler_to_rosquat(const geometry_msgs::msg::Vector3 & input)
{
  return euler_to_rosquat(input.x, input.y, input.z);
}

template <std::floating_point T>
std::tuple<double, double, double> quat_to_euler(const T & x, const T & y, const T & z, const T & w)
{
  tf2::Quaternion quat_pose;
  quat_pose.setValue(x, y, z, w);
  tf2::Matrix3x3 mat_pose(quat_pose);
  T result_x, result_y, result_z;
  mat_pose.getRPY(result_x, result_y, result_z);
  return std::make_tuple(result_x, result_y, result_z);
}

std::tuple<double, double, double> quat_to_euler(const geometry_msgs::msg::Quaternion & input)
{
  return quat_to_euler(input.x, input.y, input.z, input.w);
}

}  // namespace geometry
}  // namespace rw_util
