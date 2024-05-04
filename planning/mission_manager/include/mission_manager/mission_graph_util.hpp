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

#ifndef MISSION_GRAPH_UTIL_HPP_
#define MISSION_GRAPH_UTIL_HPP_

#include <string>
#include <algorithm>
#include <sstream>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "mission_manager/type.hpp"

namespace mission_manager
{
    class GraphUtil
    {
        public:
            void erase_space(std::string& input);
            bool get_information(const std::string& input, std::string& result);
            std::string get_key(const std::string& str_data);
            std::string get_task_name(const mission_task task);

            void show_str_graph(mission_graph_str graph, rclcpp::Logger ros_logger);
            void show_bin_graph(mission_graph_bin graph, rclcpp::Logger ros_logger);

        private:
            static bool isSpace(unsigned char c);
    };
} // namespace mission_manager

#endif