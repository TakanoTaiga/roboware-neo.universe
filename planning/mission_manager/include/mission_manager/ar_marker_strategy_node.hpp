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

#ifndef AR_MARKER_STRATEGY_NODE_HPP
#define AR_MARKER_STRATEGY_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rw_planning_msg/msg/task_action.hpp>
#include <rw_planning_msg/msg/action_result.hpp>
#include <rw_common_msgs/msg/transform_array.hpp>

namespace mission_manager
{
    class ARMarkerStrategyNode : public rclcpp::Node
    {
    public:
        explicit ARMarkerStrategyNode(const rclcpp::NodeOptions & node_options);
    private:
        rclcpp::Subscription<rw_planning_msg::msg::TaskAction>::SharedPtr sub_task_action_;
        rclcpp::Subscription<rw_common_msgs::msg::TransformArray>::SharedPtr sub_marker_;
        rclcpp::Publisher<rw_planning_msg::msg::ActionResult>::SharedPtr pub_action_result_;

        void callback_sub_task_action(const rw_planning_msg::msg::TaskAction& message);
        void callback_sub_marker(const rw_common_msgs::msg::TransformArray& message);

        bool is_task_action_get = false;
        std::vector<int> search_object_id;

        int task_id;

    };
} // namespace mission_manager

#endif