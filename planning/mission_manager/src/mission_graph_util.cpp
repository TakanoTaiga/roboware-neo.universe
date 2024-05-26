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

#include "mission_manager/mission_graph_util.hpp"

namespace mission_manager
{
    bool GraphUtil::isSpace(unsigned char c)
    {
        return std::isspace(c);
    }

    void GraphUtil::erase_space(std::string &input)
    {
        input.erase(std::remove_if(input.begin(), input.end(), isSpace), input.end());
    }

    auto GraphUtil::get_information(const std::string &input, std::string &result) -> bool
    {
        result = "";
        if (input.find("(") == std::string::npos)
        {
            return false;
        }
        std::istringstream iss(input);
        std::getline(iss, result, '(');
        std::getline(iss, result, ')');
        return true;
    }

    std::string GraphUtil::get_key(const std::string &str_data)
    {
        if (str_data.find("(") == std::string::npos)
        {
            return str_data;
        }
        std::istringstream iss(str_data);
        std::string result;
        std::getline(iss, result, '(');
        return result;
    }

    std::string GraphUtil::get_arrow(const std::string &str_data)
    {
        auto str = str_data;
        char charToRemove = '-';
        str.erase(std::remove(str.begin(), str.end(), charToRemove), str.end());
        return str;
    }

    std::string GraphUtil::str_graph_tostring(mission_graph_str graph)
    {
        std::string out_show = "show_str_graph\n";
        for (const auto &pair : graph)
        {
            out_show +=  pair.first + "(" + pair.second.infomation + ")-" + std::to_string(pair.second.id) + " connects to: ";
            for (const auto &conn : pair.second.connections)
            {
                out_show += conn + "; ";
            }
            out_show += "\n";
        }
        
        return out_show;
    }

    std::string GraphUtil::bin_graph_tostring(mission_graph_bin graph)
    {
        std::string out_show = "show_bin_graph\n";
        for (const auto &pair : graph)
        {
            out_show += std::to_string(pair.second.id) + "-" + pair.second.strategy_label + " connects to: ";
            for (const auto &conn : pair.second.connections)
            {
                out_show += std::to_string(conn) + "; ";
            }
            out_show += "\n";
        }
        return out_show;
    }
} // namespace mission_manager