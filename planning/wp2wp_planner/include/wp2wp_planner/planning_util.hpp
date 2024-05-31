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

#ifndef PLANNING_UTIL_HPP_
#define PLANNING_UTIL_HPP_

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometry.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <string>
#include <visualization_msgs/msg/marker.hpp>

namespace boost_type
{
using point_2d_lf = boost::geometry::model::d2::point_xy<double>;
using polygon_2d_lf = boost::geometry::model::polygon<point_2d_lf>;
using r_tf =
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>;
using t_tf = boost::geometry::strategy::transform::translate_transformer<double, 2, 2>;
}  // namespace boost_type

namespace wp2wp_planner
{
class PlanningUtil
{
public:
  explicit PlanningUtil();

  visualization_msgs::msg::Marker polygon_to_ros(
    std::string frame_id, builtin_interfaces::msg::Time stamp, boost_type::polygon_2d_lf & poly,
    int32_t id);

private:
};
}  // namespace wp2wp_planner

#endif