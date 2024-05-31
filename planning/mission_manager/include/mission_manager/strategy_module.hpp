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

#ifndef TASK_MODULE_HPP_
#define STRATEGY_MODULE_HPP_

#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <map>
#include <rw_common_util/geometry.hpp>
#include <rw_planning_msg/msg/task_action.hpp>
#include <sstream>
#include <string>

#include "mission_manager/type.hpp"

namespace mission_manager
{
namespace strategy_module
{
class StartStrategy : public RWStrategy
{
public:
  StartStrategy();
  void update(node_bin & node, debug_info & info) override;
};

class EndStrategy : public RWStrategy
{
public:
  EndStrategy();
  void update(node_bin & node, debug_info & info) override;
};

class SetPoseStrategy : public RWStrategy
{
public:
  SetPoseStrategy();
  void update(node_bin & node, debug_info & info) override;

private:
  point3 infomation_to_point3(const std::string & setpose_cmd);
};

class AddPoseStrategy : public RWStrategy
{
public:
  AddPoseStrategy();
  void update(node_bin & node, debug_info & info) override;

private:
};

// class FindStrategy : public RWStrategy
// {
// public:

// private:
// };
}  // namespace strategy_module
}  // namespace mission_manager

#endif