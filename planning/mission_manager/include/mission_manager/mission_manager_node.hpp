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

#ifndef MISSION_MANAGER_NODE_HPP
#define MISSION_MANAGER_NODE_HPP

#include <memory>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <rw_planning_msg/msg/task_action.hpp>
#include <rw_planning_msg/msg/action_result.hpp>

#include "mission_manager/mission_graph_module.hpp"
#include "mission_manager/state_transition.hpp"

namespace mission_manager
{
    class MissionManagerNode : public rclcpp::Node
    {
    public:
        explicit MissionManagerNode(const rclcpp::NodeOptions & node_options);
    private:
        rclcpp::Logger logger = get_logger();
        rclcpp::Subscription<rw_planning_msg::msg::ActionResult>::SharedPtr sub_action_result_;
        rclcpp::Publisher<rw_planning_msg::msg::TaskAction>::SharedPtr pub_task_action_;
        rclcpp::TimerBase::SharedPtr st_timer_;

        StateTransition state_transition_handler = StateTransition();
        
        void state_transition_callback();
        void action_result_subscriber_callback(const rw_planning_msg::msg::ActionResult& action_result);

    };
} // namespace mission_manager

#endif