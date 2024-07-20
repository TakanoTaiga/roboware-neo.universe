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

#ifndef RW_COMMON_UTIL_MAP_HPP_
#define RW_COMMON_UTIL_MAP_HPP_

#include <type_traits>
#include <utility>

namespace rw_common_util
{
    template <typename T, typename U>
    auto contains(const T& map, const U& key) 
        -> typename std::enable_if_t<
               std::is_same<
                   decltype(map.find(key)), 
                   typename T::const_iterator
               >::value, bool>
    {
        return map.find(key) != map.end();
    }
} // namespace rw_common_util

#endif