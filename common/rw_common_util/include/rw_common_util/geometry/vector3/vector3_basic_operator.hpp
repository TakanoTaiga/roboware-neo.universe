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

#ifndef RW_COMMON_UTIL__VECTOR3_BASIC_OPERATOR_HPP_
#define RW_COMMON_UTIL__VECTOR3_BASIC_OPERATOR_HPP_

#include "rw_common_util/geometry/geometry_concept.hpp"

namespace rw_util
{
namespace geometry
{

template <LikeVector3 T, LikeVector3 U>
auto operator+(const T& a, const U& b)
{
    auto v = T();
    v = a.x + b.x;
    v = a.y + b.y;
    v = a.z + b.z;
    return v;
}

template <LikeVector3 T, LikeVector3 U>
auto operator-(const T& a, const U& b)
{
    auto v = T();
    v = a.x - b.x;
    v = a.y - b.y;
    v = a.z - b.z;
    return v;
}

template <LikeVector3 T, std::floating_point U>
auto operator*(const T& a, const U& b)
{
    auto v = T();
    v = a.x * b;
    v = a.y * b;
    v = a.z * b;
    return v;
}

template <LikeVector3 T, std::floating_point U>
auto operator/(const T& a, const U& b)
{
    auto v = T();
    v = a.x / b;
    v = a.y / b;
    v = a.z / b;
    return v;
}

template <LikeVector3 T, LikeVector3 U>
auto operator+=(T& a, const U& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}

template <LikeVector3 T, LikeVector3 U>
auto operator-=(T& a, const U& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}

template <LikeVector3 T, std::floating_point U>
auto operator*=(T& a, const U& b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

template <LikeVector3 T, std::floating_point U>
auto operator/=(T& a, const U& b)
{
    a.x /= b;
    a.y /= b;
    a.z /= b;
}

}  // namespace geometry
}  // namespace rw_util

#endif
