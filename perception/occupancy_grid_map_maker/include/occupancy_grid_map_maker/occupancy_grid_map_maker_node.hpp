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

#ifndef OCCUPANCY_GRID_MAP_MAKER_NODE_HPP_
#define OCCUPANCY_GRID_MAP_MAKER_NODE_HPP_

#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "occupancy_grid_map_maker/map_maker.hpp"


namespace occupancy_grid_map_maker_node
{
    class OccupancyGridMapMakerNode : public rclcpp::Node
    {
    public:
        explicit OccupancyGridMapMakerNode(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_map;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;

        void subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_);
    
        float param_cut_z;
        int param_map_width;
        int param_map_height;
        float param_map_res;
    };
} // namespace occupancy_grid_map_maker_node

#endif