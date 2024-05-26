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
    void StateTransition::setting_callback()
    {
        strategies.push_back(new strategy_module::StartStrategy());
        strategies.push_back(new strategy_module::EndStrategy());
        strategies.push_back(new strategy_module::SetPoseStrategy());
    }
}
