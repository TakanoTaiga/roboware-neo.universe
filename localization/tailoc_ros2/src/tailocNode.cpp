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

#include "tailoc_ros2/tailocNode.hpp"
#include "tailoc_ros2/ndt_cpp.hpp"
#include "tailoc_ros2/tailoc_util.hpp"

namespace tailoc_ros2
{
    tailocNode::tailocNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("tailoc_node", node_option)
    {   
        // Publisher and Subscriber
        sub_laser_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "input/scan", 0, std::bind(&tailocNode::subscriber_callback, this, std::placeholders::_1));

        pub_current_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "output/current_pose", 0);

        // pub_path_ = create_publisher<nav_msgs::msg::Path>(
        //     "output/path", 0);

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Parameter
        ndt_param.enable_debug = declare_parameter<bool>("enable_debug" , false);
        ndt_param.ndt_max_iteration = declare_parameter<int>("ndt_max_iteration" , 10);
        ndt_param.ndt_precision = declare_parameter<double>("ndt_precision" , 1e-5);
        ndt_param.ndt_matching_step = declare_parameter<int>("ndt_matching_step" , 10);
        ndt_param.ndt_sample_num_point = declare_parameter<int>("ndt_sample_num_point" , 10);

        RCLCPP_INFO_STREAM(get_logger(),  "Initialize Task Done");
    }

    void tailocNode::subscriber_callback(const sensor_msgs::msg::LaserScan msg){
        auto start_time = std::chrono::high_resolution_clock::now();

        std::vector<ndt_cpp::point2> sensor_points;
        tailoc_util::laserscan_to_ndtcpp_point2(msg, sensor_points);
        
        if(map_points.empty()){
            std::copy(sensor_points.begin(), sensor_points.end(), std::back_inserter(map_points));
            return;
        }

        auto delta_trans_mat = ndt_cpp::makeTransformationMatrix(0.0, 0.0, 0.0);

        const auto computed_results = ndt_cpp::compute_ndt_points(ndt_param, map_points);

        ndt_cpp::ndt_scan_matching(
            ndt_param,
            get_logger(),
            delta_trans_mat,
            sensor_points,
            computed_results,
            map_points
        );

        odom.x = delta_trans_mat.c;
        odom.y = delta_trans_mat.f;
        odom.z = std::atan(delta_trans_mat.d / delta_trans_mat.a);

        const auto stamp = get_clock()->now();

        tf_broadcaster_->sendTransform(tailoc_util::point3_to_tf_stamp(odom, stamp));
  
        const auto pose = tailoc_util::point3_to_pose_stamp(odom, stamp);
        pub_current_pose_->publish(pose);

        auto end_time = std::chrono::high_resolution_clock::now(); 
        double latency_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1e6;

        if(ndt_param.enable_debug){
            RCLCPP_INFO_STREAM(get_logger(), 
                "latency " << latency_ms << "ms"
            );
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tailoc_ros2::tailocNode)