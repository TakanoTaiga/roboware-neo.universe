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

#ifndef TYPE_
#define TYPE_

#include <map>
#include <vector>
#include <string>

namespace mission_manager
{
    enum class mission_task {
        Start,
        End,
        SetPose,
        AddPose,
        Find,
        Unknown
    };

    enum class task_state {
        wait,
        start,
        working_in_progress,
        end,
        error
    };

    struct node_str {
        std::string name;
        std::string infomation;
        std::vector<std::string> connections;
        uint32_t id;
    };

    class State
    {
        public:
            State();
            bool change_state(const task_state& change_to);
            void set_error();
            void state_reset();
            task_state get_state();

        private:
            task_state state;
            std::map<task_state, task_state> allowed_transitions;
    };

    struct node_bin{
        uint32_t id;
        mission_task task;
        std::string mission_infomation;
        State state;
        bool now_transitioning;
        std::vector<uint32_t> connections;
    };

    struct point3{
        double x;
        double y;
        double z;
    };

    using mission_graph_str = std::map<std::string, node_str>;
    using mission_graph_bin = std::map<uint32_t, node_bin>;

} // namespace mission_manager

#endif