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

#ifndef POLYLOAD_HPP_
#define POLYLOAD_HPP_

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "wp2wp_planner/planning_util.hpp"

namespace wp2wp_planner
{
namespace polyload
{
    boost_type::polygon_2d_lf polyload(std::string path);
}
}

#endif