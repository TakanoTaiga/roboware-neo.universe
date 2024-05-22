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
// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <type_traits>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace rw_common_util
{
    namespace geometry
    {   
        template <typename T>
        struct rw_quat
        {
            T x, y, z, w;
        };
        template <typename T>
        struct rw_euler
        {
            T roll, pitch, yaw;
        };

        template <typename T>
        rw_quat<T> euler_to_quat(const T& roll, const T& pitch, const T& yaw);
        template <typename T>
        rw_quat<T> euler_to_quat(const rw_euler<T>& input);

        geometry_msgs::msg::Quaternion euler_to_rosquat(
            const double& roll, const double& pitch, const double& yaw);
        geometry_msgs::msg::Quaternion euler_to_rosquat(
            const rw_euler<double>& input);

        template <typename T>
        rw_euler<T> quat_to_euler(const T& x, const T& y, const T& z, const T& w);

        template <typename T>
        rw_euler<T> quat_to_euler(const rw_quat<T>& input);

        rw_euler<double> quat_to_euler(const geometry_msgs::msg::Quaternion& input);
    }
}

namespace rw_common_util
{
    namespace geometry
    {
        template <typename T, typename = void>
        struct HasMemberW : std::false_type {};

        template <typename T>
        struct HasMemberW<T, std::void_t<decltype(std::declval<T>().w)>> : std::true_type {};

        template <typename T, typename = void>
        struct IsLikeVector3 : public std::false_type
        {
        };

        template <typename T>
        struct IsLikeVector3<
        T, std::void_t<
            decltype(std::declval<T>().x),
            decltype(std::declval<T>().y),
            decltype(std::declval<T>().z),
            std::enable_if_t<!HasMemberW<T>::value>
        >>
        : public std::true_type
        {
        };

        template <typename T, typename = void>
        struct IsLikeQuaternion : std::false_type {};

        template <typename T>
        struct IsLikeQuaternion<
        T, std::void_t<
            decltype(std::declval<T>().x),
            decltype(std::declval<T>().y),
            decltype(std::declval<T>().z),
            decltype(std::declval<T>().w)
        >>
        : std::true_type
        {
        };

    }  // namespace geometry
}  // namespace rw_common_util

namespace rw_common_util
{
  namespace geometry
  {
    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, IsLikeVector3<U>>, std::nullptr_t> =
        nullptr>
    auto operator+(const T & a, const U & b)
    {
      if constexpr (std::is_same<T, geometry_msgs::msg::Vector3>::value)
      {
        geometry_msgs::msg::Vector3 v;
        v.x = a.x + b.x;
        v.y = a.y + b.y;
        v.z = a.z + b.z;
        return v;
      }
      else
      {
        geometry_msgs::msg::Point v;
        v.x = a.x + b.x;
        v.y = a.y + b.y;
        v.z = a.z + b.z;
        return v;
      }
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, IsLikeVector3<U>>, std::nullptr_t> =
        nullptr>
    auto operator-(const T & a, const U & b)
    {
      if constexpr (std::is_same<T, geometry_msgs::msg::Vector3>::value)
      {
        geometry_msgs::msg::Vector3 v;
        v.x = a.x - b.x;
        v.y = a.y - b.y;
        v.z = a.z - b.z;
        return v;
      }
      else
      {
        geometry_msgs::msg::Point v;
        v.x = a.x - b.x;
        v.y = a.y - b.y;
        v.z = a.z - b.z;
        return v;
      }
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, std::is_scalar<U>>, std::nullptr_t> =
        nullptr>
    auto operator*(const T & a, const U & b)
    {
      if constexpr (std::is_same<T, geometry_msgs::msg::Vector3>::value)
      {
        geometry_msgs::msg::Vector3 v;
        v.x = a.x * b;
        v.y = a.y * b;
        v.z = a.z * b;
        return v;
      }
      else
      {
        geometry_msgs::msg::Point v;
        v.x = a.x * b;
        v.y = a.y * b;
        v.z = a.z * b;
        return v;
      }
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, std::is_scalar<U>>, std::nullptr_t> =
        nullptr>
    auto operator/(const T & a, const U & b)
    {
      if constexpr (std::is_same<T, geometry_msgs::msg::Vector3>::value)
      {
        geometry_msgs::msg::Vector3 v;
        v.x = a.x / b;
        v.y = a.y / b;
        v.z = a.z / b;
        return v;
      }
      else
      {
        geometry_msgs::msg::Point v;
        v.x = a.x / b;
        v.y = a.y / b;
        v.z = a.z / b;
        return v;
      }
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, IsLikeVector3<U>>, std::nullptr_t> =
        nullptr>
    auto operator+=(T & a, const U & b) -> decltype(auto)
    {
      a.x += b.x;
      a.y += b.y;
      a.z += b.z;
      return a;
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, std::is_scalar<U>>, std::nullptr_t> =
        nullptr>
    auto operator*=(const T & a, const U & b)
    {
      a.x *= b;
      a.y *= b;
      a.z *= b;
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, std::is_scalar<U>>, std::nullptr_t> =
        nullptr>
    auto operator/=(const T & a, const U & b)
    {
      a.x /= b;
      a.y /= b;
      a.z /= b;
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>>, std::nullptr_t> =
        nullptr>
    auto operator+(const T & a, const U & b)
    {
      geometry_msgs::msg::Quaternion v;
      v.x = a.x + b.x;
      v.y = a.y + b.y;
      v.z = a.z + b.z;
      v.w = a.w + b.w;
      return v;
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>>, std::nullptr_t> =
        nullptr>
    auto operator-(const T & a, const U & b)
    {
      geometry_msgs::msg::Quaternion v;
      v.x = a.x - b.x;
      v.y = a.y - b.y;
      v.z = a.z - b.z;
      v.w = a.w - b.w;
      return v;
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>>, std::nullptr_t> =
        nullptr>
    auto operator*(const T & a, const U & b)
    {
      geometry_msgs::msg::Quaternion v;
      v.x = a.w  * b.x - a.z * b.y + a.y * b.z + a.x * b.w;
      v.y = a.z  * b.x + a.w * b.y - a.x * b.z + a.y * b.w;
      v.z = -a.y * b.x + a.x * b.y + a.w * b.z + a.z * b.w;
      v.w = -a.x * b.x - a.y * b.y - a.z * b.z + a.w * b.w;
      return v;
    }

    template <
      typename T, typename U,
      std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>>, std::nullptr_t> =
        nullptr>
    auto operator+=(T & a, const U & b) -> decltype(auto)
    {
      a.x += b.x;
      a.y += b.y;
      a.z += b.z;
      a.w += b.w;
      return a;
    }
  }  // namespace geometry
}  // namespace rw_common_util



namespace rw_common_util
{
  namespace geometry
  {
    geometry_msgs::msg::Twist operator+(const geometry_msgs::msg::Twist& a, const geometry_msgs::msg::Twist& b)
    {
      geometry_msgs::msg::Twist v;
      v.angular = a.angular + b.angular;
      v.linear  = a.linear  + b.linear;
      return v;
    }

    geometry_msgs::msg::Twist operator-(const geometry_msgs::msg::Twist& a, const geometry_msgs::msg::Twist& b)
    {
      geometry_msgs::msg::Twist v;
      v.angular = a.angular - b.angular;
      v.linear  = a.linear  - b.linear;
      return v;
    }

    auto operator+=(geometry_msgs::msg::Twist& a, const geometry_msgs::msg::Twist& b)
    {
      a.angular += b.angular;
      a.linear  += b.linear;
    }

    template <
      typename T,
      std::enable_if_t<std::conjunction_v<std::is_scalar<T>>, std::nullptr_t> =
        nullptr>
    auto operator*=(geometry_msgs::msg::Twist& a, const T& b)
    {
      a.angular *= b.angular;
      a.linear  *= b.linear;
    }

    template <
      typename T,
      std::enable_if_t<std::conjunction_v<std::is_scalar<T>>, std::nullptr_t> =
        nullptr>
    auto operator/=(geometry_msgs::msg::Twist& a, const T& b)
    {
      a.angular /= b.angular;
      a.linear  /= b.linear;
    }

    template <
      typename T,
      std::enable_if_t<std::conjunction_v<std::is_scalar<T>>, std::nullptr_t> =
        nullptr>
    geometry_msgs::msg::Twist operator*(const geometry_msgs::msg::Twist& a, const T& b)
    {
      geometry_msgs::msg::Twist v;
      v.angular = a.angular * b;
      v.linear  = a.linear  * b;
      return v;
    }

    template <
      typename T,
      std::enable_if_t<std::conjunction_v<std::is_scalar<T>>, std::nullptr_t> =
        nullptr>
    geometry_msgs::msg::Twist operator/(const geometry_msgs::msg::Twist& a, const T& b)
    {
      geometry_msgs::msg::Twist v;
      v.angular = a.angular / b;
      v.linear  = a.linear  / b;
      return v;
    }
  }
}