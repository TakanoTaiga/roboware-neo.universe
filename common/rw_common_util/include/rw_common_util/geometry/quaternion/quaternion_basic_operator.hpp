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

#ifndef RW_COMMON_UTIL__QUATERNION_BASIC_OPERATOR_HPP_
#define RW_COMMON_UTIL__QUATERNION_BASIC_OPERATOR_HPP_

#include "rw_common_util/geometry/geometry_concept.hpp"

namespace rw_util
{
namespace geometry
{

template <LikeQuaternion T, LikeQuaternion U>
auto operator+(const T & a, const U & b)
{
  auto v = T();
  v.x = a.x + b.x;
  v.y = a.y + b.y;
  v.z = a.z + b.z;
  v.w = a.w + b.w;
  return v;
}

template <LikeQuaternion T, LikeQuaternion U>
auto operator-(const T & a, const U & b)
{
  auto v = T();
  v.x = a.x - b.x;
  v.y = a.y - b.y;
  v.z = a.z - b.z;
  v.w = a.w - b.w;
  return v;
}

template <LikeQuaternion T, LikeQuaternion U>
auto operator*(const T & a, const U & b)
{
  auto v = T();
  v.x = a.w * b.x - a.z * b.y + a.y * b.z + a.x * b.w;
  v.y = a.z * b.x + a.w * b.y - a.x * b.z + a.y * b.w;
  v.z = -a.y * b.x + a.x * b.y + a.w * b.z + a.z * b.w;
  v.w = -a.x * b.x - a.y * b.y - a.z * b.z + a.w * b.w;
  return v;
}

template <LikeQuaternion T, LikeQuaternion U>
auto operator+=(T & a, const U & b)
{
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  a.w += b.w;
}

}  // namespace geometry
}  // namespace rw_util

#endif
