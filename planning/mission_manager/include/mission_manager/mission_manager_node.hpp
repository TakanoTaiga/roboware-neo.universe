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

#ifndef MISSION_MANAGER_NODE_HPP
#define MISSION_MANAGER_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rw_planning_msg/srv/motion_command.hpp>
#include <rw_planning_msg/srv/robot_start.hpp>
#include <rw_planning_msg/srv/robot_stop.hpp>
#include <rw_common_msgs/msg/status.hpp>

#include "mission_manager/mission_graph_module.hpp"
#include "mission_manager/task_module_util.hpp"

namespace mission_manager
{
    class MissionManagerNode : public rclcpp::Node, public MissionGraph
    {
    public:
        explicit MissionManagerNode(const rclcpp::NodeOptions & node_options);
    private:
        rclcpp::Service<rw_planning_msg::srv::MotionCommand>::SharedPtr motion_command_;
        rclcpp::Service<rw_planning_msg::srv::RobotStop>::SharedPtr robot_move_event_closer_;
        rclcpp::Service<rw_planning_msg::srv::RobotStart>::SharedPtr robot_move_stater_;
    };
} // namespace mission_manager

#endif