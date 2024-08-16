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

#include "thrown_object_estimator/thrown_object_estimator_node.hpp"

namespace thrown_object_estimator
{
    thrown_object_estimator_node::thrown_object_estimator_node(const rclcpp::NodeOptions &node_option)
    : rclcpp::Node("thrown_object_estimator_node", node_option)
    {   
       sub_detection_result = create_subscription<rw_common_msgs::msg::TransformArray>(
            "input/object_poses", 0, std::bind(&thrown_object_estimator_node::subscriber_callback, this, std::placeholders::_1));
    }

    void thrown_object_estimator_node::subscriber_callback(const rw_common_msgs::msg::TransformArray& msg)
    {
        for(const auto& pose: msg.transforms)
        {
            const auto& tf = pose.transform.translation;
            if(tf.z < 0.5){continue;}
            if(tf.z < 1.0){continue;}
            std::cout << tf.x << ", " << tf.y  << ", "<< tf.z << std::endl;
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(thrown_object_estimator::thrown_object_estimator_node)
