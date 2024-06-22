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

#include "mission_manager/state_transition.hpp"

namespace mission_manager
{
    StateTransition::StateTransition()
    {
        setting_callback();
    }

    void StateTransition::set_graph(const mission_graph_bin& input_graph)
    {
        node_graph = input_graph;
    }

    void StateTransition::get_task_action_publisher(rclcpp::Publisher<rw_planning_msg::msg::TaskAction>::SharedPtr publisher)
    {
        for(auto& strategy : strategies)
        {
            strategy->get_action_publisher(publisher);
        }
    }

    void StateTransition::get_action_result(const rw_planning_msg::msg::ActionResult& action_result_msg)
    {
        action_results[action_result_msg.task_id] = action_result_msg;
    }

    debug_info StateTransition::state_transition_master()
    {
        mission_manager::debug_info info;

        for (auto& [id, node] : node_graph)
        {
            if (!node.now_transitioning) continue;

            node.if_result = if_statement::Unknown;

            for (const auto& strategy : strategies)
            {
                if (strategy->is_match_strategy_label(node.strategy_label))
                {
                    strategy->get_action_result(action_results[node.id]);
                    strategy->update(node, info);
                }
            }
            update_graph(id);
            return info;
        }
        return info;
    }

    void StateTransition::update_graph(uint32_t id)
    {
        if(node_graph[id].state.get_state() != state_transition_label::end) return;

        node_graph[id].now_transitioning = false;
        action_results[id] = rw_planning_msg::msg::ActionResult();
        
        for(const auto& [start_node_id, ifstate] : node_graph[id].connections)
        {
            if(ifstate != node_graph[id].if_result){ continue; }
            node_graph[start_node_id].now_transitioning = true;
            node_graph[start_node_id].state.change_state(state_transition_label::start);
        }
    }

    bool StateTransition::is_end()
    {
        for (const auto& [id, node] : node_graph)
        {
            if (node.strategy_label.find("END") != std::string::npos && node.state.get_state() == state_transition_label::end)
            {
                return true;
            }
        }
        return false;
    }
}