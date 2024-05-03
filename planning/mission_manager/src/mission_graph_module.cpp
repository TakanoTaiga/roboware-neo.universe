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

                auto source_key = get_key(source);
                auto target_key = get_key(target);

                graph_str[source_key].name = source_key;
                graph_str[source_key].connections.push_back(target_key);

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
        for (const auto &pair : graph_str)
        {
            graph_str[pair.first].id = id_iter;
            id_iter++;
        }
    }

    mission_graph_bin MissionGraph::get_bin_graph(mission_graph_str &graph_str)
    {
        mission_graph_bin graph_bin;

        for (const auto &pair : graph_str)
        {
            // set id
            graph_bin[pair.second.id].id = pair.second.id;

            // set connections
            for (const auto &connection : pair.second.connections)
            {
                graph_bin[pair.second.id].connections.push_back(graph_str[connection].id);
            }

            // set task
            if (pair.second.infomation.find("START") != std::string::npos)
            {
                graph_bin[pair.second.id].task = mission_task::Start;
            }
            else if (pair.second.infomation.find("END") != std::string::npos)
            {
                graph_bin[pair.second.id].task = mission_task::End;
            }
            else if (pair.second.infomation.find("SETPOSE") != std::string::npos)
            {
                graph_bin[pair.second.id].task = mission_task::SetPose;
            }
            else if (pair.second.infomation.find("ADDPOSE") != std::string::npos)
            {
                graph_bin[pair.second.id].task = mission_task::AddPose;
            }
            else if (pair.second.infomation.find("FIND") != std::string::npos)
            {
                graph_bin[pair.second.id].task = mission_task::Find;
            }
            else
            {
                graph_bin[pair.second.id].task = mission_task::Unknown;
            }
        }
        return graph_bin;
    }
} // namespace mission_manager