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

#ifndef RWSIMPLE_PLANNING_SIMULATOR_NODE_HPP_
#define RWSIMPLE_PLANNING_SIMULATOR_NODE_HPP_

#include <vector>
#include <memory>
#include <chrono>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rw_common_util/geometry.hpp>

#include "rw_simple_planning_simulator/planning_simulator_util.hpp"
#include "rw_simple_planning_simulator/ar_marker_simulation.hpp"

namespace rw_simple_planning_simulator
{
    class RWSimplePlanningSimulatorNode : public rclcpp::Node
    {
    public:
        explicit RWSimplePlanningSimulatorNode(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr tf_timer_;

        std::vector<geometry_msgs::msg::Twist> twist_history_;

        geometry_msgs::msg::Twist twist_msg;
        geometry_msgs::msg::TransformStamped tf_stamp;
        double roll;

        double transform_noise_strength;
        double rotation_noise_strength;
        double rotation_tr_noise_strength;

        std::random_device seed_gen;
        std::default_random_engine engine;
        std::normal_distribution<> dist_transform;
        std::normal_distribution<> dist_rotation;

        ar_marker_simulation armkr_sim;

        void timer_callback();
        void subscriber_callback(const geometry_msgs::msg::Twist& msg);
    };
} // namespace rw_simple_planning_simulator

#endif // RWSIMPLE_PLANNING_SIMULATOR_NODE_HPP_
