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

#include "mission_manager/mission_manager_node.hpp"

namespace mission_manager
{
    MissionManagerNode::MissionManagerNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("MissionManagerNode", node_option)
    {   
        //log file initialize
        std::ofstream log_file;
        log_file.open("/tmp/rw.log");
        log_file.close();
        // Graph initialize
        const auto file_path = declare_parameter<std::string>("graphpath" , "./graph.md");
        auto gen_mission_graph = MissionGraph();
        gen_mission_graph.get_mission_graph(file_path);
        state_transition_handler.set_graph(gen_mission_graph.at_mgraph_bin());

        // get end mode
        const auto end_mode = declare_parameter<std::string>("endmode" , "kill"); //kill or safe
        if(end_mode == "kill"){is_end_killallnode = true;}
        else{is_end_killallnode = false;}
        
        // Graph visualize
        RCLCPP_INFO_STREAM(logger, gen_mission_graph.str_graph_tostring(gen_mission_graph.at_mgraph_str()));
        RCLCPP_INFO_STREAM(logger, gen_mission_graph.bin_graph_tostring(gen_mission_graph.at_mgraph_bin()));

        // ROS 2 initialize
        pub_task_action_ = create_publisher<rw_planning_msg::msg::TaskAction>(
            "output/task_action", 0);
        state_transition_handler.get_task_action_publisher(pub_task_action_);

        sub_action_result_ = create_subscription<rw_planning_msg::msg::ActionResult>(
            "input/action_result", 0, std::bind(&MissionManagerNode::action_result_subscriber_callback, this, std::placeholders::_1));

        st_timer_  = 
            create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MissionManagerNode::state_transition_callback, this));

    }

    void MissionManagerNode::state_transition_callback()
    {
        const auto info = state_transition_handler.state_transition_master();
        RCLCPP_INFO_STREAM(logger, info.debug_str);

        if(state_transition_handler.is_end()){
            if(is_end_killallnode){
                const auto _ = system("ps aux | grep ros | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh");
            }else{
                rclcpp::shutdown();
            }

        }
    }

    void MissionManagerNode::action_result_subscriber_callback(const rw_planning_msg::msg::ActionResult& action_result)
    {
        state_transition_handler.get_action_result(action_result);
    }

} // namespace mission_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mission_manager::MissionManagerNode)