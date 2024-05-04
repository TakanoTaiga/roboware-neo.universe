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
        std::string file_path = "/home/taiga/ros_ws/src/roboware-neo.universe/s-graph.md";
        auto gen_mission_graph = MissionGraph();
        gen_mission_graph.get_mission_graph(file_path);

        // visualize
        RCLCPP_INFO_STREAM(logger, gen_mission_graph.str_graph_tostring(gen_mission_graph.at_mgraph_str()));

        RCLCPP_INFO_STREAM(logger, gen_mission_graph.bin_graph_tostring(gen_mission_graph.at_mgraph_bin()));

        state_transition_handler.set_graph(gen_mission_graph.at_mgraph_bin());
        std::cout << std::endl << "StateTransition" << std::endl;
        state_transition_handler.state_transition_master();
        state_transition_handler.state_transition_master();
        state_transition_handler.state_transition_master();
        state_transition_handler.state_transition_master();
        state_transition_handler.state_transition_master();

        std::exit(0);
    }
} // namespace mission_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mission_manager::MissionManagerNode)