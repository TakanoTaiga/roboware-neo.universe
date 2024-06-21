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

#include "mission_manager/mission_graph_module.hpp"

namespace mission_manager
{
    void MissionGraph::get_mission_graph(const std::string& file_path)
    {
        try
        {
            get_str_graph(file_path, mgraph_str);
        }
        catch (const std::runtime_error &e)
        {
            std::cout << e.what() << std::endl;
            std::exit(1);
        }
        
        get_bin_graph(mgraph_str, mgraph_bin);
    }

    mission_graph_str MissionGraph::at_mgraph_str()
    {
        if(mgraph_str.empty()) throw std::runtime_error("Access empty str graph.");
        return mgraph_str;
    }
    
    mission_graph_bin MissionGraph::at_mgraph_bin()
    {
        if(mgraph_bin.empty()) throw std::runtime_error("Access empty bin graph.");
        return mgraph_bin;
    }

    void MissionGraph::get_str_graph(const std::string &file_path, mission_graph_str &graph_str)
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open the file. path=" + file_path);
        }
        std::string line;
        while (std::getline(file, line))
        {
            if (line.find("graph TB") != std::string::npos || line.empty())
            {
                continue;
            }
            std::istringstream iss(line);
            std::string source, arrow, target;
            if (
                std::getline(iss, source, '-') &&
                std::getline(iss, arrow, '>') &&
                std::getline(iss, target))
            {
                erase_space(source);
                erase_space(arrow);
                erase_space(target);

                const auto source_key = get_key(source);
                const auto target_key = get_key(target);
                const auto formatted_arrow = get_arrow(arrow);

                graph_str[source_key].name = source_key;
                graph_str[source_key].connections.push_back(std::make_pair(target_key, formatted_arrow));

                graph_str[target_key].name = target_key;

                std::string result;
                if (get_information(source, result))
                    graph_str[source_key].infomation = result;
                if (get_information(target, result))
                    graph_str[target_key].infomation = result;
            }
        }
        // add id
        uint32_t id_iter = 0;
        for (const auto& pair : graph_str)
        {
            graph_str[pair.first].id = id_iter;
            id_iter++;
        }
    }

    void MissionGraph::get_bin_graph(mission_graph_str& graph_str, mission_graph_bin& result_mgraph)
    {
        for (const auto &pair : graph_str)
        {
            // set id
            result_mgraph[pair.second.id].id = pair.second.id;
            result_mgraph[pair.second.id].state = State();
            result_mgraph[pair.second.id].now_transitioning = false;

            // set connections
            for (const auto &[connection , ifstate] : pair.second.connections)
            {
                // std::cout << ifstate << std::endl;
                result_mgraph[pair.second.id].connections.push_back(
                   std::make_pair((graph_str[connection].id), str_to_ifstatement(ifstate)));
            }

            result_mgraph[pair.second.id].strategy_label = pair.second.infomation;

            // set task
            if (pair.second.infomation.find("START") != std::string::npos)
            {
                result_mgraph[pair.second.id].state.change_state(state_transition_label::start);
                result_mgraph[pair.second.id].now_transitioning = true;
            }

            // copy mission infomations
            result_mgraph[pair.second.id].mission_infomation = pair.second.infomation;

        }
    }
} // namespace mission_manager