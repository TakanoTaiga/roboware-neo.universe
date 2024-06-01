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

#ifndef RW_COMMON_DEMO_NODE_HPP_
#define RW_COMMON_DEMO_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rw_common_util/geometry.hpp"

namespace rw_common_util
{
    class RWCommonUtilNode : public rclcpp::Node
    {
    public:
        explicit RWCommonUtilNode(const rclcpp::NodeOptions & node_options);
    private:
    };
} // namespace rw_common_util

#endif