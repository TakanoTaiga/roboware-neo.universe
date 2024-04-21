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
    
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        pub_tf_timer_  = 
            create_wall_timer(std::chrono::milliseconds(20), std::bind(&SimplePlanningSimulatorNode::timer_callback, this));

        tf_stamp = geometry_msgs::msg::TransformStamped();
        tf_stamp.header.frame_id = "map";
        tf_stamp.child_frame_id = "base_link";
        twist_msg = geometry_msgs::msg::Twist();
    }

    void SimplePlanningSimulatorNode::timer_callback(){
        // twist linear: m/s
        // twist anguler: rad/s

        tf_stamp.header.stamp = now();

        tf_stamp.transform.translation.x += twist_msg.linear.x * 0.02;
        tf_stamp.transform.translation.y += twist_msg.linear.y * 0.02;
        tf_stamp.transform.translation.z = 0.0;

        roll += twist_msg.angular.z * 0.002 * 57.295 * -1.0;

        tf2::Quaternion q;
        q.setEuler(0.0, 0.0, roll);
        tf_stamp.transform.rotation.x = q.x();
        tf_stamp.transform.rotation.y = q.y();
        tf_stamp.transform.rotation.z = q.z();
        tf_stamp.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_stamp);
    }

    void SimplePlanningSimulatorNode::subscriber_callback(const geometry_msgs::msg::Twist& msg){
        twist_msg = msg;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_planning_simulator::SimplePlanningSimulatorNode)