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

#include "mcu_ros_driver/mcu_util.hpp"

namespace mcu_util
{
    std::pair<std::string, std::string> mcu_info_parser(const std::vector<uint8_t>& input)
    {
        std::string recv_data_str(input.begin(), input.end());
        std::string addr, name;
        std::stringstream ss(recv_data_str);
        std::getline(ss, addr, ',');
        std::getline(ss, name, ',');
        return std::make_pair(addr, name);
    }
} // namespace mcu_util

