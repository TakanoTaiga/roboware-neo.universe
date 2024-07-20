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

#include "depth_merge_node/util.hpp"

namespace depth_merge_node
{
namespace util
{
    key_time get_key(const builtin_interfaces::msg::Time& msg)
    {
        return std::make_pair(msg.sec, msg.nanosec);
    }

    key_time get_key(const std_msgs::msg::Header& msg)
    {
        return get_key(msg.stamp);
    }
}
}
