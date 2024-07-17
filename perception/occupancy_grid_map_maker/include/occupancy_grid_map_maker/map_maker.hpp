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

#ifndef MAP_MAKER_HPP_
#define MAP_MAKER_HPP_

#include <memory>
#include <chrono>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "cv_bridge/cv_bridge.h"

namespace map_maker
{
    nav_msgs::msg::OccupancyGrid make_occupancy_msg(
        const int width, const int height, const float res
    );

    void point_to_map(
        nav_msgs::msg::OccupancyGrid& map,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const float cut_z
    );

    cv::Mat occupancy_grid_to_cv_image(const nav_msgs::msg::OccupancyGrid& map);
} // namespace map_maker

#endif