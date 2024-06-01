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

#ifndef RW_COMMON_UTIL__QUATERNION_UTILL_HPP_
#define RW_COMMON_UTIL__QUATERNION_UTILL_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tuple>

#include "rw_common_util/geometry/geometry_concept.hpp"

namespace rw_util {
namespace geometry {

template <std::floating_point T>
auto quat_to_euler(const T & x, const T & y, const T & z, const T & w) -> std::tuple<T, T, T>;

}  // namespace geometry
}  // namespace rw_util

#endif
