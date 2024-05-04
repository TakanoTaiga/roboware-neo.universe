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
#include "mission_manager/mission_graph_module.hpp"

#include "mission_manager/state_transition.hpp"

namespace mission_manager
{
    MissionManagerNode::MissionManagerNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("MissionManagerNode", node_option)
    {
        std::string file_path = "/home/taiga/ros_ws/src/roboware-neo.universe/s-graph.md";

        get_mission_graph(file_path);

        // visualize
        std::cout << std::endl << "show_str_graph" << std::endl;
        show_str_graph(at_mgraph_str());

        std::cout << std::endl << "show_bin_graph" << std::endl;
        show_bin_graph(at_mgraph_bin());

        auto st = StateTransition(at_mgraph_bin());
        std::cout << std::endl << "StateTransition" << std::endl;
        st.state_transition_master();
        st.state_transition_master();
        st.state_transition_master();
        st.state_transition_master();
        st.state_transition_master();

        std::exit(0);
    }
} // namespace mission_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mission_manager::MissionManagerNode)