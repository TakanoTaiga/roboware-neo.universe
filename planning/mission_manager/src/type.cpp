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
  state = state_transition_label::wait;

  allowed_transitions[state_transition_label::wait] = state_transition_label::start;
  allowed_transitions[state_transition_label::start] = state_transition_label::working_in_progress;
  allowed_transitions[state_transition_label::working_in_progress] = state_transition_label::end;
  allowed_transitions[state_transition_label::end] = state_transition_label::start;
}

bool State::change_state(const state_transition_label & change_to)
{
  if (allowed_transitions[state] == change_to) {
    state = change_to;
    return true;
  }
  std::cout << "ERROR_STATE_CHANGE" << std::endl;
  return false;
}

void State::set_error() { state = state_transition_label::error; }

void State::state_reset() { state = state_transition_label::wait; }

state_transition_label State::get_state() const { return state; }
}  // namespace mission_manager

namespace mission_manager
{
namespace strategy_module
{
void RWStrategy::get_action_publisher(
  rclcpp::Publisher<rw_planning_msg::msg::TaskAction>::SharedPtr publisher)
{
  pub_task_action_ = publisher;
}

void RWStrategy::get_action_result(const rw_planning_msg::msg::ActionResult & action_result_msg)
{
  action_result = action_result_msg;
}

bool RWStrategy::is_match_strategy_label(const std::string & label)
{
  return label.find(strategy_label) != std::string::npos;
}
}  // namespace strategy_module
}  // namespace mission_manager