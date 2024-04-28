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

#include "simple_planning_simulator/SimplePlanningSimulatorNode.hpp"

namespace simple_planning_simulator
{
    SimplePlanningSimulatorNode::SimplePlanningSimulatorNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("Simple_planning_simulatorNode", node_option)
    {   
        sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
            "input/cmd_vel", 0, std::bind(&SimplePlanningSimulatorNode::subscriber_callback, this, std::placeholders::_1));

        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "output/pose", 0);
    
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_timer_  = 
            create_wall_timer(std::chrono::milliseconds(20), std::bind(&SimplePlanningSimulatorNode::timer_callback, this));

        transform_noise_strength = declare_parameter<double>("noise.transoform.strength" , 0.02);
        transform_noise_sd = declare_parameter<double>("noise.transoform.sd" , 0.2);
        rotation_noise_strength = declare_parameter<double>("noise.rotation.strength" , 0.002);
        rotation_tr_noise_strength = declare_parameter<double>("noise.rotation_tr.strength" , 0.01);
        rotation_noise_sd = declare_parameter<double>("noise.rotation.sd" , 0.5);

        tf_stamp = geometry_msgs::msg::TransformStamped();
        tf_stamp.header.frame_id = declare_parameter<std::string>("frameid.map" , "map");
        tf_stamp.child_frame_id = declare_parameter<std::string>("frameid.base_link" , "base_link");
        twist_msg = geometry_msgs::msg::Twist();
    }

    void SimplePlanningSimulatorNode::timer_callback(){
        twist_history.push_back(twist_msg);
        if(twist_history.size() >= 10){
            twist_history.erase(twist_history.begin());
        }

        auto twist_avg = geometry_msgs::msg::Twist();

        for(const auto& twist : twist_history){
            twist_avg.linear.x += twist.linear.x;
            twist_avg.linear.y += twist.linear.y;
            twist_avg.linear.z += twist.linear.z;

            twist_avg.angular.x += twist.angular.x;
            twist_avg.angular.y += twist.angular.y;
            twist_avg.angular.z += twist.angular.z;
        }
        twist_avg.linear.x /= twist_history.size();
        twist_avg.linear.y /= twist_history.size();
        twist_avg.linear.z /= twist_history.size();

        twist_avg.angular.x /= twist_history.size();
        twist_avg.angular.y /= twist_history.size();
        twist_avg.angular.z /= twist_history.size();

        std::random_device seed_gen;
        std::default_random_engine engine(seed_gen());
        std::normal_distribution<> dist_tf(0.0, transform_noise_sd);
        std::normal_distribution<> dist_4r(0.0, rotation_noise_sd);


        // twist linear: m/s
        // twist anguler: rad/s
        //dt = 0.02

        tf_stamp.header.stamp = now();

        tf_stamp.transform.translation.x += twist_avg.linear.x * 0.02 + twist_avg.linear.x * 0.02 * transform_noise_strength * dist_tf(engine);
        tf_stamp.transform.translation.y += twist_avg.linear.y * 0.02 + twist_avg.linear.y * 0.02 * transform_noise_strength * dist_tf(engine);
        tf_stamp.transform.translation.z = 0.0;

        roll += twist_avg.angular.z * 0.02 * -1.0 +
            twist_avg.angular.z * 0.02 * rotation_noise_strength * dist_tf(engine) +
            twist_avg.linear.x * 0.02 * dist_4r(engine) * rotation_tr_noise_strength +
            twist_avg.linear.y * 0.02 * dist_4r(engine) * rotation_tr_noise_strength;

        tf2::Quaternion q;
        q.setEuler(0.0, 0.0, roll);
        tf_stamp.transform.rotation.x = q.x();
        tf_stamp.transform.rotation.y = q.y();
        tf_stamp.transform.rotation.z = q.z();
        tf_stamp.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_stamp);

        auto pose_msg = geometry_msgs::msg::PoseStamped();

        pose_msg.header = tf_stamp.header;

        pose_msg.pose.position.x = tf_stamp.transform.translation.x;
        pose_msg.pose.position.y = tf_stamp.transform.translation.y;
        pose_msg.pose.position.z = tf_stamp.transform.translation.z;

        pose_msg.pose.orientation.x = tf_stamp.transform.rotation.x;
        pose_msg.pose.orientation.y = tf_stamp.transform.rotation.y;
        pose_msg.pose.orientation.z = tf_stamp.transform.rotation.z;
        pose_msg.pose.orientation.w = tf_stamp.transform.rotation.w;
        pub_pose_->publish(pose_msg);
    }

    void SimplePlanningSimulatorNode::subscriber_callback(const geometry_msgs::msg::Twist& msg){
        twist_msg = msg;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_planning_simulator::SimplePlanningSimulatorNode)