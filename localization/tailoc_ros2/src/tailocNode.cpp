// Copyright 2023 Hakoroboken
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

namespace tailoc_ros2
{
    tailocNode::tailocNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("tailoc_node", node_option)
    {   
        // Publisher and Subscriber
        sub_laser_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 0, std::bind(&tailocNode::subscriber_callback, this, std::placeholders::_1));

        pub_current_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "output/current_pose", 0);

        pub_path_ = create_publisher<nav_msgs::msg::Path>(
            "output/path", 0);

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Parameter
        ndt_param.enable_debug = declare_parameter<bool>("enable_debug" , false);
        ndt_param.ndt_max_iteration = declare_parameter<int>("ndt_max_iteration" , 10);
        ndt_param.ndt_precision = declare_parameter<double>("ndt_precision" , 1e-5);
        ndt_param.ndt_matching_step = declare_parameter<int>("ndt_matching_step" , 10);
        ndt_param.ndt_sample_num_point = declare_parameter<int>("ndt_sample_num_point" , 10);
        ndt_param.ndt_initial_pose_x = declare_parameter<double>("ndt_initial_pose_x" , 0.0);
        ndt_param.ndt_initial_pose_y = declare_parameter<double>("ndt_initial_pose_y" , 0.0);
        ndt_param.ndt_initial_pose_rad = declare_parameter<double>("ndt_initial_pose_rad" , 0.0);
    }

    void tailocNode::subscriber_callback(const sensor_msgs::msg::LaserScan msg){
        static std::vector<ndt_cpp::point2> before_points;

        std::vector<ndt_cpp::point2> sensor_points;
        for(size_t i=0; i < msg.ranges.size(); ++i) {
            ndt_cpp::point2 lp;
            double th = msg.angle_min + msg.angle_increment * i;
            double r = msg.ranges[i];
            if (msg.range_min < r && r < msg.range_max) {
                lp.x = r * cos(th); lp.y = r * sin(th);
                sensor_points.push_back(lp);
            }
        }

        if(before_points.size() < 10){
            for(const auto& point : sensor_points){
                before_points.push_back(point);
            }
            return;
        }

        auto trans_mat = ndt_cpp::makeTransformationMatrix(
            ndt_param.ndt_initial_pose_x,
            ndt_param.ndt_initial_pose_y,
            ndt_param.ndt_initial_pose_rad
        );

        const auto covs = ndt_cpp::compute_ndt_points(ndt_param, before_points);

        ndt_cpp::ndt_scan_matching(
            ndt_param,
            trans_mat,
            sensor_points,
            before_points,
            covs
        );

        odom.x += trans_mat.c;
        odom.y += trans_mat.f;
        odom.z += atan(trans_mat.d / trans_mat.a);

        before_points.clear();
        for(const auto& point : sensor_points){
            before_points.push_back(point);
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        t.transform.translation.x = odom.x;
        t.transform.translation.y = odom.y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, odom.z);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = t.header;

        pose.pose.position.x = odom.x;
        pose.pose.position.y = odom.y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        pub_current_pose_->publish(pose);

        path.header = t.header;
        path.poses.push_back(pose);
        pub_path_->publish(path);

    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tailoc_ros2::tailocNode)