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

#ifndef RW_COMMON_UTIL__CONCEPT_HPP_
#define RW_COMMON_UTIL__CONCEPT_HPP_

#include <concepts>

namespace rw_util
{
namespace geometry
{

template <typename T>
concept HasMemberW = requires(T a)
{
  a.w;
};

template <typename T>
concept LikeVector3 = requires(T a)
{
  a.x;
  a.y;
  a.z;
}
&&!HasMemberW<T>;

template <typename T>
concept LikeQuaternion = requires(T a)
{
  a.x;
  a.y;
  a.z;
  a.w;
};

}  // namespace geometry
}  // namespace rw_util

#endif
