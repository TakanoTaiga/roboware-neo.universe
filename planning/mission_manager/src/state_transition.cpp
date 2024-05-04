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
        strategies[mission_task::Start] = new task_module::StartStrategy();
        strategies[mission_task::End] = new task_module::EndStrategy();
        strategies[mission_task::SetPose] = new task_module::SetPoseStrategy();
    }

    void StateTransition::set_graph(const mission_graph_bin& input_graph)
    {
        node_graph = input_graph;
    }

    void StateTransition::state_transition_master()
    {
        for (auto& pair : node_graph) {
            if (!pair.second.now_transitioning) continue;

            if (strategies.find(pair.second.task) != strategies.end()) {
                strategies[pair.second.task]->update(pair.second);
            }

            update_graph(pair.first);
            return;
        }
    }

    void StateTransition::update_graph(uint32_t id)
    {
        if(node_graph[id].state.get_state() != task_state::end) return;

        node_graph[id].now_transitioning = false;
        for(const auto& start_node_id : node_graph[id].connections)
        {
            node_graph[start_node_id].now_transitioning = true;
            node_graph[start_node_id].state.change_state(task_state::start);
        }
    }
}