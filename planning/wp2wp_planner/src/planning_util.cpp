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

#include "wp2wp_planner/planning_util.hpp"

namespace wp2wp_planner
{
PlanningUtil::PlanningUtil() {}

visualization_msgs::msg::Marker PlanningUtil::polygon_to_ros(
  std::string frame_id, builtin_interfaces::msg::Time stamp, boost_type::polygon_2d_lf & poly,
  int32_t id)
{
  auto marker_msg = visualization_msgs::msg::Marker();
  marker_msg.header.stamp = stamp;
  marker_msg.header.frame_id = frame_id;
  marker_msg.ns = "polygon";
  marker_msg.id = id;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.scale.x = 0.05;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;

  for (const auto & point : poly.outer()) {
    auto ros_point = geometry_msgs::msg::Point();
    ros_point.x = point.x();
    ros_point.y = point.y();
    marker_msg.points.push_back(ros_point);
  }

  return marker_msg;
}
}  // namespace wp2wp_planner