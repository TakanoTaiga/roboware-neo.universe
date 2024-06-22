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
    // private
    bool GraphUtil::isSpace(unsigned char c)
    {
        return std::isspace(c);
    }

    // public
    void GraphUtil::erase_space(std::string &input)
    {
        input.erase(std::remove_if(input.begin(), input.end(), isSpace), input.end());
    }

    void GraphUtil::to_lower_case(std::string& str) {
        std::transform(str.begin(), str.end(), str.begin(),
                    [](unsigned char c) { return std::tolower(c); });
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

    if_statement GraphUtil::str_to_ifstatement(const std::string& str_data)
    {
        auto data = str_data;
        erase_space(data);
        to_lower_case(data);

        std::cout << "str_to_ifstatement: " << data  << std::endl;

        if(data.find("true") != std::string::npos)
        {
            return if_statement::True;
        }
            
        if(data.find("false") != std::string::npos)
        {
            return if_statement::False;
        }

        return if_statement::Unknown;
    }

    std::string GraphUtil::str_graph_tostring(mission_graph_str graph)
    {
        std::string out_show = "show_str_graph\n";
        for (const auto &pair : graph)
        {
            out_show +=  pair.first + "(" + pair.second.infomation + ")-" + std::to_string(pair.second.id) + " connects to: ";
            for (const auto &[connection, ifstate] : pair.second.connections)
            {
                out_show += "(" + connection + ", " + ifstate + "), ";
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
            for (const auto &[connection, ifstate] : pair.second.connections)
            {
                out_show += std::to_string(connection) + "(" + enum_to_string(ifstate) +"); ";
            }
            out_show += "\n";
        }
        return out_show;
    }

    std::string GraphUtil::enum_to_string(if_statement input)
    {
        if(input == if_statement::True)
            return "true";
        else if(input == if_statement::False)
            return "false";
        else
            return "unknown";
    }
}

namespace mission_util
{
    info_decode::info_decode(const std::string& infomation)
    {
        infomation_ = infomation;
    }

    std::map<std::string, std::string> info_decode::decode_str()
    {
        std::string param_type, param_name, param_var;
        std::istringstream iss(infomation_.substr(infomation_.find(':') + 1));
        std::string token;

        std::map<std::string, std::string> results;

        while (std::getline(iss, token, ',')) {
            size_t pos = token.find('=');
            if (pos != std::string::npos) {
                std::string key = token.substr(0, pos);
                std::string value = token.substr(pos + 1);
                results[key] = value;
            }
        }

        return results;
    }
    
    std::map<std::string, double> info_decode::decode_double()
    {
        std::map<std::string, double> results;

        const auto data_str = decode_str();
        for(const auto& [key, value] : data_str)
        {
            results[key] = std::stod(value);
        }

        return results;
    }

    std::map<std::string, int> info_decode::decode_int()
    {
        std::map<std::string, int> results;

        const auto data_str = decode_str();
        for(const auto& [key, value] : data_str)
        {
            results[key] = std::stoi(value);
        }

        return results;
    }
} // namespace mission_manager
