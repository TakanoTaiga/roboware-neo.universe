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

#ifndef MISSION_GRAPH_MODULE_HPP_
#define MISSION_GRAPH_MODULE_HPP_

#include <fstream>
#include <stdexcept>

#include "mission_manager/mission_graph_util.hpp"
#include "mission_manager/type.hpp"

namespace mission_manager
{
class MissionGraph : public GraphUtil
{
public:
  void get_mission_graph(const std::string & file_path);
  mission_graph_str at_mgraph_str();
  mission_graph_bin at_mgraph_bin();

private:
  void get_str_graph(const std::string & file_path, mission_graph_str & graph_str);
  void get_bin_graph(mission_graph_str & graph_str, mission_graph_bin & result_mgraph);
  mission_graph_str mgraph_str;
  mission_graph_bin mgraph_bin;
};
}  // namespace mission_manager

#endif