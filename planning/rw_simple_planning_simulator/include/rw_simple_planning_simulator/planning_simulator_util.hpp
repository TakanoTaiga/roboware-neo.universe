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

#ifndef PLANNING_SIMULATOR_UTILL_HPP_
#define PLANNING_SIMULATOR_UTILL_HPP_

#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rw_common_util/geometry.hpp>

namespace rw_simple_planning_simulator
{
namespace planning_util
{
    geometry_msgs::msg::Twist calculate_average_twist(const std::vector<geometry_msgs::msg::Twist>& twist_history);
} // namespace planning_util
} // rw_simple_planning_simulator

#endif // PLANNING_SIMULATOR_UTILL_HPP_
