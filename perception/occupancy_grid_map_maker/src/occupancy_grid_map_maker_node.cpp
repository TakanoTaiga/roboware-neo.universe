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

#include "occupancy_grid_map_maker/occupancy_grid_map_maker_node.hpp"

namespace occupancy_grid_map_maker_node
{
    OccupancyGridMapMakerNode::OccupancyGridMapMakerNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("depth_merger_node", node_option)
    {   
        pub_occupancy_map = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "output/occupancy_map", 0);
        pub_debug_image_ = create_publisher<sensor_msgs::msg::Image>(
            "debug/image", 0);
        sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/velodyne/velodyne_points", 0, std::bind(&OccupancyGridMapMakerNode::subscriber_callback, this, std::placeholders::_1));
    
        param_cut_z = declare_parameter<float>("cut_z" , -0.2);
        param_map_res = declare_parameter<float>("map.res" , 0.1);
        param_map_width = declare_parameter<int>("map.width" , 100);
        param_map_height = declare_parameter<int>("map.height" , 100);
    }

    void OccupancyGridMapMakerNode::subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg_, *clouds);

        auto map_msg = map_maker::make_occupancy_msg(param_map_width, param_map_height, param_map_res);
        map_maker::point_to_map(map_msg, clouds, param_cut_z);
        map_msg.header.frame_id = msg_->header.frame_id;
        map_msg.header.stamp = get_clock()->now();
        pub_occupancy_map->publish(map_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_map_maker_node::OccupancyGridMapMakerNode)