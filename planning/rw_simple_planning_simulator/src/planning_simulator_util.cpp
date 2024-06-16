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

#include "rw_simple_planning_simulator/planning_simulator_util.hpp"

namespace rw_simple_planning_simulator
{
namespace planning_util
{
    geometry_msgs::msg::Twist calculate_average_twist(const std::vector<geometry_msgs::msg::Twist>& twist_history)
    {
        using rw_common_util::geometry::operator+=;
        using rw_common_util::geometry::operator/=;

        auto twist_avg = geometry_msgs::msg::Twist();

        for (const auto& twist : twist_history) {
            twist_avg.linear += twist.linear;
            twist_avg.angular += twist.angular;
        }

        twist_avg.linear /= twist_history.size();
        twist_avg.angular /= twist_history.size();

        if (!std::isfinite(twist_avg.angular.z)) {
            twist_avg.angular.z = 0.0;
        }
        if (!std::isfinite(twist_avg.linear.x)) {
            twist_avg.linear.x = 0.0;
        }
        if (!std::isfinite(twist_avg.linear.y)) {
            twist_avg.linear.y = 0.0;
        }

        return twist_avg;
    }
}
}