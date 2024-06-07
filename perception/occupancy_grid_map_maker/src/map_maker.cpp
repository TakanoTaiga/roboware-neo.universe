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

#include "occupancy_grid_map_maker/map_maker.hpp"

namespace map_maker
{
    nav_msgs::msg::OccupancyGrid make_occupancy_msg(
        const int width, const int height, const float res
    ){
        auto message = nav_msgs::msg::OccupancyGrid();
        message.info.resolution = res;
        message.info.width = width;
        message.info.height = height;
        message.info.origin.position.x = -width  * res / 2.0;
        message.info.origin.position.y = -height * res / 2.0;
        message.info.origin.position.z = -2.0;
        return message;
    }

    void point_to_map(
        nav_msgs::msg::OccupancyGrid& map,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const float cut_z
    ){
        const auto map_area_size = map.info.height * map.info.width;
        if(map.data.size() < map_area_size)
        {
            map.data.resize(map_area_size);
        }

        const auto& map_pos_x = map.info.origin.position.x;
        const auto& map_pos_y = map.info.origin.position.y;
        const auto& map_res = map.info.resolution;
        
        const auto& map_width = map.info.width;
        const auto& map_height = map.info.height;

        for(const auto& point : cloud->points){
            if(point.z < cut_z){continue;}

            const auto grid_x = static_cast<int>((point.x - map_pos_x) / map_res);
            const auto grid_y = static_cast<int>((point.y - map_pos_y) / map_res);

            if (grid_x < 0 || grid_x >= map_width || grid_y < 0 || grid_y >= map_height) {
                continue;
            }

            const auto index = grid_y * map_width + grid_x;
            if (index < 0 || index >= map.data.size()) {
                continue;
            }
            map.data[index] = 100;
        }
    }
}