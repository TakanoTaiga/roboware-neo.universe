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

#ifndef PATH_FOLLOWER_NODE_HPP_
#define PATH_FOLLOWER_NODE_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rw_common_util/geometry.hpp>
#include <rw_planning_msg/msg/action_result.hpp>
#include <rw_planning_msg/msg/task_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace path_follower
{
enum planner_status { ready, not_found };

class PathFollowerNode : public rclcpp::Node
{
public:
  explicit PathFollowerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_current_pose_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_nav_path_;
  rclcpp::Subscription<rw_planning_msg::msg::TaskAction>::SharedPtr sub_task_action_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<rw_planning_msg::msg::ActionResult>::SharedPtr pub_action_result;

  rclcpp::TimerBase::SharedPtr control_timer_;

  geometry_msgs::msg::PoseStamped current_pose;
  nav_msgs::msg::Path global_path;
  planner_status pose_status;
  planner_status path_status;
  size_t start_pose_index;
  size_t target_pose_index;
  geometry_msgs::msg::PoseStamped start_pose;
  geometry_msgs::msg::PoseStamped goal_pose;

  int32_t task_id;

  void timer_callback();
  void current_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped & sub_msg_pose);
  void nav_path_subscriber_callback(const nav_msgs::msg::Path & sub_msg_path);
  void task_action_subscriber_callback(const rw_planning_msg::msg::TaskAction & action_msg);
};
}  // namespace path_follower

#endif