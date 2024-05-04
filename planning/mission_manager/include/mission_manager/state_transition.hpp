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

namespace mission_manager
{
    class StateTransition
    {
    public:
        StateTransition(const mission_graph_bin& input_graph);
        void state_transition_master();
    private:
        mission_graph_bin node_graph;
        void update_graph(uint32_t id, task_state sate);
    };
} // namespace mission_manager

#endif