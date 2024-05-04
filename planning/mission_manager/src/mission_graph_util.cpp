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

    std::string GraphUtil::get_task_name(const mission_task task)
    {
        if (task == mission_task::Start)
        {
            return "Start";
        }
        else if (task == mission_task::End)
        {
            return "End";
        }
        else if (task == mission_task::SetPose)
        {
            return "SetPose";
        }
        else if (task == mission_task::AddPose)
        {
            return "AddPose";
        }
        else if (task == mission_task::Find)
        {
            return "Find";
        }
        return "Unknown";
    }

    void GraphUtil::show_str_graph(mission_graph_str graph, rclcpp::Logger ros_logger)
    {
        std::string out_show = "show_str_graph\n";
        for (const auto &pair : graph)
        {
            out_show +=  pair.first + "(" + pair.second.infomation + ")-" + std::to_string(pair.second.id) + " connects to: ";
            // std::cout << pair.first << "(" << pair.second.infomation << ")-" << pair.second.id << " connects to: ";
            for (const auto &conn : pair.second.connections)
            {
                // std::cout << conn << "; ";
                out_show += conn + "; ";
            }
            out_show += "\n";
            // std::cout << std::endl;
        }
        RCLCPP_INFO_STREAM(ros_logger, out_show);
        // std::cout << std::endl;
    }

    void GraphUtil::show_bin_graph(mission_graph_bin graph, rclcpp::Logger ros_logger)
    {
        std::string out_show = "show_bin_graph\n";
        for (const auto &pair : graph)
        {
            out_show += std::to_string(pair.second.id) + "-" + get_task_name(pair.second.task) + " connects to: ";
            for (const auto &conn : pair.second.connections)
            {
                out_show += std::to_string(conn) + "; ";
            }
            out_show += "\n";
        }
        RCLCPP_INFO_STREAM(ros_logger, out_show);
    }
} // namespace mission_manager