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
        StartStrategy::StartStrategy()
        {
            strategy_label = "START";
        }

        void StartStrategy::update(node_bin& node, debug_info& info)
        {
            info.debug_str = "Start:" + node.mission_infomation;
            node.state.change_state(state_transition_label::working_in_progress);
            node.state.change_state(state_transition_label::end);
        }

        EndStrategy::EndStrategy()
        {
            strategy_label = "END";
        }

        void EndStrategy::update(node_bin& node, debug_info& info)
        {
            info.debug_str = "End:" + node.mission_infomation;
            node.state.change_state(state_transition_label::working_in_progress);
            node.state.change_state(state_transition_label::end);
        }

        SetPoseStrategy::SetPoseStrategy()
        {
            strategy_label = "SETPOSE";
        }

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

                const auto rad = p3.z * 0.017453292519;
                pub_msg.pose.pose.orientation = rw_common_util::geometry::euler_to_rosquat(0.0, 0.0, rad);

                pub_task_action_->publish(pub_msg);

                node.state.change_state(state_transition_label::working_in_progress);
            }
            else if(node.state.get_state() == state_transition_label::working_in_progress)
            {
                info.debug_str = "working in progress";
                if(action_result.status.code == rw_common_msgs::msg::Status::SUCCESS && action_result.status.success)
                {
                    const auto p3 = infomation_to_point3(node.mission_infomation);
                    node.state.change_state(state_transition_label::end);
                    std::ofstream log_file;
                    log_file.open("/tmp/rw.log", std::ios::app);
                    log_file << "target , " << std::to_string(p3.x) << "," << std::to_string(p3.y) << "," << std::to_string(p3.z) << std::endl;
                    log_file.close();
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

        FindStrategy::FindStrategy()
        {
            strategy_label = "FIND";
        }

        void FindStrategy::update(node_bin& node, debug_info& info)
        {
            if(node.state.get_state() == state_transition_label::start)
            {
                std::tie(param_type, param_name, param_var) = infomation_to_findorder(node.mission_infomation);
                std::cout << param_type << ", " << param_name << ", " << param_var << std::endl;

                auto pub_msg = rw_planning_msg::msg::TaskAction();

                pub_msg.task = rw_planning_msg::msg::TaskAction::FIND;
                pub_msg.id = node.id;

                auto obj = rw_planning_msg::msg::FindObject();
                obj.object_name = param_name;
                obj.object_type = param_type;
                pub_msg.object.push_back(obj);

                pub_task_action_->publish(pub_msg);

                node.state.change_state(state_transition_label::working_in_progress);
                start_time = std::chrono::system_clock::now();
            }
            else if(node.state.get_state() == state_transition_label::working_in_progress)
            {
                const auto end_time = std::chrono::system_clock::now();
                const auto millsec = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                if(millsec < 500){ return; }


                bool is_getmarker = false;
                for(const auto& object : action_result.result_object)
                {
                    if(object.object_name != param_name){ continue; }

                    is_getmarker = true;
                    
                    std::cout << object.object_name << std::endl;
                }
                node.if_result = is_getmarker ? if_statement::True : if_statement::False;

                node.state.change_state(state_transition_label::end);
            }
        }

        std::tuple<std::string, std::string, std::string> FindStrategy::infomation_to_findorder(const std::string& findcmd)
        {
            std::string param_type, param_name, param_var;
            std::istringstream iss(findcmd.substr(findcmd.find(':') + 1));
            std::string token;

            std::map<std::string, std::string*> coord_map = {
                {"type", &param_type},
                {"name", &param_name},
                {"var", &param_var}
            };

            while (std::getline(iss, token, ',')) {
                size_t pos = token.find('=');
                if (pos != std::string::npos) {
                    std::string key = token.substr(0, pos);
                    std::string value = token.substr(pos + 1);
                    auto it = coord_map.find(key);
                    if (it != coord_map.end()) {
                        *(it->second) = value;
                    } else {
                        throw std::runtime_error("Unknown Key Error: " + key);
                    }
                }
            }

            return std::make_tuple(param_type, param_name, param_var);
        }

    }
}