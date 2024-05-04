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
#define TASK_MODULE_HPP_

#include <string>
#include <map>
#include <sstream>
#include <iostream>

#include "mission_manager/type.hpp"

namespace mission_manager
{
namespace task_module
{
    class StartStrategy : public TaskStrategy
    {
    public:
        void update(node_bin& node) override;
    };

    class EndStrategy : public TaskStrategy
    {
    public:
        void update(node_bin& node) override;
    };

    class SetPoseStrategy : public TaskStrategy
    {
    public:
        void update(node_bin& node) override;
    private:
        point3 infomation_to_point3(const std::string& setpose_cmd);
    };

    class AddPoseStrategy : public TaskStrategy
    {
    public:
         
    private:
    };

    class FindStrategy : public TaskStrategy
    {
    public:
         
    private:
    };
}
} // namespace mission_manager

#endif