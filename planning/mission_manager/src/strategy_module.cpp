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

#include "mission_manager/strategy_module.hpp"

namespace mission_manager
{
namespace strategy_module
{
    void StartStrategy::update(node_bin& node, debug_info& info)
    {
        info.debug_str = "Start:" + node.mission_infomation;
        node.state.change_state(state_transition_label::working_in_progress);
        node.state.change_state(state_transition_label::end);
    }
}
}

namespace mission_manager
{
namespace strategy_module
{
    void EndStrategy::update(node_bin& node, debug_info& info)
    {
        info.debug_str = "End:" + node.mission_infomation;
        node.state.change_state(state_transition_label::working_in_progress);
        node.state.change_state(state_transition_label::end);
    }
}
}

namespace mission_manager
{
namespace strategy_module
{
    void SetPoseStrategy::update(node_bin& node, debug_info& info)
    {
        if(node.state.get_state() == state_transition_label::start)
        {
            const auto p3 = infomation_to_point3(node.mission_infomation);
            info.debug_str = std::to_string(p3.x) + "," + std::to_string(p3.y) + "," + std::to_string(p3.z);

            auto pub_msg = rw_planning_msg::msg::TaskAction();
            pub_msg.header.frame_id = "map";

            pub_msg.task = rw_planning_msg::msg::TaskAction::SETPOSE;
            pub_msg.id = node.id;

            pub_msg.pose.pose.position.x = p3.x;
            pub_msg.pose.pose.position.y = p3.y;
            pub_msg.pose.pose.position.z = 0.0;

            pub_task_action_->publish(pub_msg);

            node.state.change_state(state_transition_label::working_in_progress);

        }
        else if(node.state.get_state() == state_transition_label::working_in_progress)
        {
            info.debug_str = "working in progress";
            if(action_result.status.code == rw_common_msgs::msg::Status::SUCCESS && action_result.status.success)
            {
                node.state.change_state(state_transition_label::end);
            }
        }
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