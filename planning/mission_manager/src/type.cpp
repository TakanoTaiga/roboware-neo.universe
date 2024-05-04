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

#include "mission_manager/type.hpp"

namespace mission_manager
{
    State::State()
    {
        state = task_state::wait;

        allowed_transitions[task_state::wait] = task_state::start;
        allowed_transitions[task_state::start] = task_state::working_in_progress;
        allowed_transitions[task_state::working_in_progress] = task_state::end;
        allowed_transitions[task_state::end] = task_state::start;
    }

    bool State::change_state(const task_state& change_to)
    {
        if (allowed_transitions[state] == change_to) {
            state = change_to;
            return true;
        }
        return false;
    }

    void State::set_error()
    {
        state = task_state::error;
    }

    void State::state_reset()
    {
        state = task_state::wait;
    }

    task_state State::get_state()
    {
        return state;
    }
}