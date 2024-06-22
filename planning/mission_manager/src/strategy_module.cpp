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
            auto decoded_info_double = mission_util::info_decode(node.mission_infomation).decode_double();
            const auto point_x = decoded_info_double["x"];
            const auto point_y = decoded_info_double["y"];
            const auto point_z = decoded_info_double["z"];

            if(node.state.get_state() == state_transition_label::start)
            {
                info.debug_str = std::to_string(point_x) + "," + std::to_string(point_y) + "," + std::to_string(point_z);

                auto pub_msg = rw_planning_msg::msg::TaskAction();
                pub_msg.header.frame_id = "map";

                pub_msg.task = rw_planning_msg::msg::TaskAction::SETPOSE;
                pub_msg.id = node.id;

                pub_msg.pose.pose.position.x = point_x;
                pub_msg.pose.pose.position.y = point_y;
                pub_msg.pose.pose.position.z = 0.0;

                const auto rad = point_z * 0.017453292519;
                pub_msg.pose.pose.orientation = rw_common_util::geometry::euler_to_rosquat(0.0, 0.0, rad);

                pub_task_action_->publish(pub_msg);

                node.state.change_state(state_transition_label::working_in_progress);
            }
            else if(node.state.get_state() == state_transition_label::working_in_progress)
            {
                info.debug_str = "working in progress";
                if(action_result.status.code == rw_common_msgs::msg::Status::SUCCESS && action_result.status.success)
                {
                    node.state.change_state(state_transition_label::end);
                    std::ofstream log_file;
                    log_file.open("/tmp/rw.log", std::ios::app);
                    log_file << "target , " << std::to_string(point_x) << "," << std::to_string(point_y) << "," << std::to_string(point_z) << std::endl;
                    log_file.close();
                }
            }
        }

        FindStrategy::FindStrategy()
        {
            strategy_label = "FIND";
        }

        void FindStrategy::update(node_bin& node, debug_info& info)
        {
            auto decoded_info_double = mission_util::info_decode(node.mission_infomation).decode_str();
            const auto param_type = decoded_info_double["type"];
            const auto param_name = decoded_info_double["name"];
            const auto param_var  = decoded_info_double["var"];

            if(node.state.get_state() == state_transition_label::start)
            {
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

        WaitStrategy::WaitStrategy()
        {
            strategy_label = "WAIT";
        }

        void WaitStrategy::update(node_bin& node, debug_info& info)
        {
            auto decoded_info_double = mission_util::info_decode(node.mission_infomation).decode_int();
            const auto param_time = decoded_info_double["millsec"];

            if(node.state.get_state() == state_transition_label::start)
            {
                start_time = std::chrono::system_clock::now();
                node.state.change_state(state_transition_label::working_in_progress);
            }
            else if(node.state.get_state() == state_transition_label::working_in_progress)
            {
                const auto end_time = std::chrono::system_clock::now();
                const auto millsec = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                if(millsec < param_time){return;}
                node.state.change_state(state_transition_label::end);
            }
        }
    }
}
