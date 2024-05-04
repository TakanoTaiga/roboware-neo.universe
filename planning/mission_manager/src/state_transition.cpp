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
    StateTransition::StateTransition(const mission_graph_bin& input_graph)
    {
        node_graph = input_graph;
    }

    void StateTransition::state_transition_master()
    {
        for(auto& pair : node_graph)
        {
            if(!pair.second.now_transitioning) continue;

            task_state continued_task_sate;

            if(pair.second.task == mission_task::Start)
            {
                pair.second.state.change_state(task_state::start);
                pair.second.state.change_state(task_state::working_in_progress);
                continued_task_sate = task_state::end;

                std::cout << "Start: " << pair.second.mission_infomation << std::endl;
            }
            else if(pair.second.task == mission_task::End)
            {
                pair.second.state.change_state(task_state::start);
                pair.second.state.change_state(task_state::working_in_progress);
                continued_task_sate = task_state::end;
                std::cout << "End: " << pair.second.mission_infomation << std::endl;
            }
            else if(pair.second.task == mission_task::SetPose)
            {
                pair.second.state.change_state(task_state::start);
                pair.second.state.change_state(task_state::working_in_progress);
                continued_task_sate = task_state::end;
                std::cout << "SetPose: " << pair.second.mission_infomation << std::endl;
            }
            else if(pair.second.task == mission_task::AddPose)
            {
                
            }
            else if(pair.second.task == mission_task::Find)
            {
                
            }
            update_graph(pair.first, continued_task_sate);
            return;
        }
    }

    void StateTransition::update_graph(uint32_t id, task_state sate)
    {
        if(sate != task_state::end) return;

        node_graph[id].now_transitioning = false;
        node_graph[id].state.change_state(task_state::end);
        for(const auto& start_node_id : node_graph[id].connections)
        {
            node_graph[start_node_id].now_transitioning = true;
            node_graph[start_node_id].state.change_state(task_state::start);
        }
    }
}