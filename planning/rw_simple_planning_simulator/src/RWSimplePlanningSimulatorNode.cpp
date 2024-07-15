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
#include "rw_simple_planning_simulator/RWSimplePlanningSimulatorNode.hpp"

namespace rw_simple_planning_simulator
{
    RWSimplePlanningSimulatorNode::RWSimplePlanningSimulatorNode(const rclcpp::NodeOptions &node_options)
        : rclcpp::Node("Simple_planning_simulatorNode", node_options),
          engine(seed_gen()),
          dist_transform(0.0, declare_parameter<double>("noise.transform.sd", 0.2)),
          dist_rotation(0.0, declare_parameter<double>("noise.rotation.sd", 0.5)),
          transform_noise_strength(declare_parameter<double>("noise.transform.strength", 0.02)),
          rotation_noise_strength(declare_parameter<double>("noise.rotation.strength", 0.002)),
          rotation_tr_noise_strength(declare_parameter<double>("noise.rotation_tr.strength", 0.01)),
          real_mode(declare_parameter<bool>("sim.realmode", false)),
          tf_stamp(),
          roll(0.0),
          armkr_sim(
            1.500983,
            20,
            0.4
          )
    {
        for(auto i = 0; i < 100; ++i)
        {
            if(declare_parameter<bool>("enable_marker_id." + std::to_string(i), false))
            {
                armkr_sim.set_param(declare_parameter<std::string>("armarker." + std::to_string(i), ""), i);
            }
        }

        sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
            "input/cmd_vel", 10, std::bind(&RWSimplePlanningSimulatorNode::subscriber_callback, this, std::placeholders::_1));

        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "output/pose", 10);

        pub_marker_realtime_ = create_publisher<rw_common_msgs::msg::TransformArray>(
            "output/marker", 10);

        pub_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "output/debug", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&RWSimplePlanningSimulatorNode::timer_callback, this));

        tf_stamp.transform.translation.x = declare_parameter<double>("init_pose.x", -0.9);
        tf_stamp.transform.translation.y = declare_parameter<double>("init_pose.y", 0.9);
        tf_stamp.header.frame_id = declare_parameter<std::string>("frameid.map", "map");
        tf_stamp.child_frame_id = declare_parameter<std::string>("frameid.base_link", "base_link");
        twist_msg = geometry_msgs::msg::Twist();
    }

    void RWSimplePlanningSimulatorNode::timer_callback()
    {
        twist_history_.push_back(twist_msg);
        if (twist_history_.size() >= 10) {
            twist_history_.erase(twist_history_.begin());
        }

        auto twist_avg = planning_util::calculate_average_twist(twist_history_);

        tf_stamp.header.stamp = now();

        tf_stamp.transform.translation.x += twist_avg.linear.x * 0.02 + twist_avg.linear.x * 0.02 * transform_noise_strength * dist_transform(engine);
        tf_stamp.transform.translation.y += twist_avg.linear.y * 0.02 + twist_avg.linear.y * 0.02 * transform_noise_strength * dist_transform(engine);
        tf_stamp.transform.translation.z = 0.0;

        roll += twist_avg.angular.z * 0.02 * -1.0 +
                twist_avg.angular.z * 0.02 * rotation_noise_strength * dist_transform(engine) +
                twist_avg.linear.x * 0.02 * dist_rotation(engine) * rotation_tr_noise_strength +
                twist_avg.linear.y * 0.02 * dist_rotation(engine) * rotation_tr_noise_strength;

        tf_stamp.transform.rotation = rw_common_util::geometry::euler_to_rosquat(0.0, 0.0, roll);

        tf_broadcaster_->sendTransform(tf_stamp);

        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header = tf_stamp.header;
        pose_msg.pose.position.x = tf_stamp.transform.translation.x;
        pose_msg.pose.position.y = tf_stamp.transform.translation.y;
        pose_msg.pose.position.z = tf_stamp.transform.translation.z;
        pose_msg.pose.orientation = tf_stamp.transform.rotation;

        pub_pose_->publish(pose_msg);

        armkr_sim.set_current_pose(pose_msg.pose);

        auto viz_msg = armkr_sim.pub_pose(tf_broadcaster_, pub_marker_realtime_, now());
        viz_msg.markers.push_back(polygon_to_ros("map", now(), armkr_sim.detection_area, 1));
        pub_debug_->publish(viz_msg);
    }

    void RWSimplePlanningSimulatorNode::subscriber_callback(const geometry_msgs::msg::Twist& msg)
    {
        if(real_mode)
        {
            twist_msg.linear.x = msg.linear.x * cos(roll) - msg.linear.y * sin(roll);
            twist_msg.linear.y = msg.linear.x * sin(roll) + msg.linear.y * cos(roll);
            twist_msg.angular.z = msg.angular.z;
        }else
        {
            twist_msg = msg;
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rw_simple_planning_simulator::RWSimplePlanningSimulatorNode)
