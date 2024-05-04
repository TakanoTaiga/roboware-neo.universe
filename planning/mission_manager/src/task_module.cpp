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

#include "mission_manager/task_module.hpp"

namespace mission_manager
{
namespace task_module
{
    void StartStrategy::update(node_bin& node)
    {
        std::cout << "Start:" << node.mission_infomation << std::endl;
        node.state.change_state(task_state::start);
        node.state.change_state(task_state::working_in_progress);
        node.state.change_state(task_state::end);
    }
}
}

namespace mission_manager
{
namespace task_module
{
    void EndStrategy::update(node_bin& node)
    {
        std::cout << "End:" << node.mission_infomation << std::endl;
        node.state.change_state(task_state::start);
        node.state.change_state(task_state::working_in_progress);
        node.state.change_state(task_state::end);
    }
}
}

namespace mission_manager
{
namespace task_module
{
    void SetPoseStrategy::update(node_bin& node)
    {
        node.state.change_state(task_state::start);
        node.state.change_state(task_state::working_in_progress);
        node.state.change_state(task_state::end);

        const auto p3 = infomation_to_point3(node.mission_infomation);
        std::cout << p3.x << "," << p3.y << "," << p3.z << ","  << std::endl;
    }

    point3 SetPoseStrategy::infomation_to_point3(const std::string& setpose_cmd)
    {
        point3 result_point;
        std::istringstream iss(setpose_cmd.substr(setpose_cmd.find(':') + 1));
        std::string token;

        std::map<std::string, double*> coord_map = {
            {"x", &result_point.x},
            {"y", &result_point.y},
            {"z", &result_point.z}
        };

        while (std::getline(iss, token, ',')) {
            size_t pos = token.find('=');
            if (pos != std::string::npos) {
                std::string key = token.substr(0, pos);
                double value = std::stod(token.substr(pos + 1));
                auto it = coord_map.find(key);
                if (it != coord_map.end()) {
                    *(it->second) = value;
                } else {
                    throw std::runtime_error("Unknown Key Error: " + key);
                }
            }
        }
       
        return result_point;
    }
}
}