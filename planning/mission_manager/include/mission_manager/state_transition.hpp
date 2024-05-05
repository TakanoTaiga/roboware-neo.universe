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

#ifndef STATE_TRANSITION_HPP_
#define STATE_TRANSITION_HPP_

#include <optional>
#include <map>
#include <iostream>

#include "mission_manager/type.hpp"
#include "mission_manager/task_module.hpp"

namespace mission_manager
{
    class StateTransition
    {
    public:
        StateTransition();
        void set_graph(const mission_graph_bin& input_graph);
        void get_task_action_publisher(rclcpp::Publisher<rw_planning_msg::msg::TaskAction>::SharedPtr publisher);
        void get_action_result(const rw_planning_msg::msg::ActionResult& action_result_msg);
        debug_info state_transition_master();
        bool is_end();
    private:
        mission_graph_bin node_graph;
        void update_graph(uint32_t id);

        //task modules
        std::map<mission_task, task_module::TaskStrategy*> strategies;
        std::map<uint32_t, rw_planning_msg::msg::ActionResult> action_results;
    };
} // namespace mission_manager

#endif