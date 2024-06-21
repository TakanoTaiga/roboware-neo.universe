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

#include "mission_manager/ar_marker_strategy_node.hpp"

namespace mission_manager
{
    ARMarkerStrategyNode::ARMarkerStrategyNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("ARMarkerStrategyNode", node_option)
    {
        pub_action_result_ = create_publisher<rw_planning_msg::msg::ActionResult>(
            "output/action_result", 0);

        sub_task_action_ = create_subscription<rw_planning_msg::msg::TaskAction>(
            "input/task_action", 0, std::bind(&ARMarkerStrategyNode::callback_sub_task_action, this, std::placeholders::_1));

        sub_marker_ = create_subscription<rw_common_msgs::msg::TransformArray>(
            "input/marker", 0, std::bind(&ARMarkerStrategyNode::callback_sub_marker, this, std::placeholders::_1));
    }

    void ARMarkerStrategyNode::callback_sub_task_action(const rw_planning_msg::msg::TaskAction& message)
    {
        if(message.task != rw_planning_msg::msg::TaskAction::FIND){ return; }

        search_object_id.clear();
        for(const auto& object: message.object)
        {
            if(object.object_type.find("armarker") == std::string::npos){ continue; }
            search_object_id.push_back(std::stoi(object.object_name));
            is_task_action_get = true;
        }
        task_id = message.id;
    }

    void ARMarkerStrategyNode::callback_sub_marker(const rw_common_msgs::msg::TransformArray& message)
    {
        if(!is_task_action_get){ return; }
        is_task_action_get = false;

        auto pub_msg = rw_planning_msg::msg::ActionResult();
        pub_msg.status.code = rw_common_msgs::msg::Status::SUCCESS;
        pub_msg.status.success = true;  
        pub_msg.task_id = task_id;

        std::map<int, std::vector<geometry_msgs::msg::TransformStamped>> finded_list;

        for(const auto& id : search_object_id)
        {
            for(const auto& transform : message.transforms)
            {
                const auto marker_id = std::stoi(transform.child_frame_id);
                if(marker_id != id){ continue; }

                finded_list[id].push_back(transform);
            }
        }

        for(const auto& [id, tf_msgs] :  finded_list)
        {
            auto obj_msg = rw_planning_msg::msg::FindObject();
            obj_msg.object_name = std::to_string(id);
            obj_msg.object_type = "armarker";
            for(const auto& tf_msg : tf_msgs)
            {
                obj_msg.pose.push_back(tf_msg);
            }
            pub_msg.result_object.push_back(obj_msg);
        }     

        pub_action_result_->publish(pub_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mission_manager::ARMarkerStrategyNode)