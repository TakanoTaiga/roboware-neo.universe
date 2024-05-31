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

#include "path_follower/PathFollowerNode.hpp"

namespace path_follower
{
PathFollowerNode::PathFollowerNode(const rclcpp::NodeOptions & node_option)
: rclcpp::Node("PathFollowerNode", node_option)
{
  sub_current_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/current_pose", 0,
    std::bind(&PathFollowerNode::current_pose_subscriber_callback, this, std::placeholders::_1));
  sub_nav_path_ = create_subscription<nav_msgs::msg::Path>(
    "input/nav_path", 0,
    std::bind(&PathFollowerNode::nav_path_subscriber_callback, this, std::placeholders::_1));
  sub_task_action_ = create_subscription<rw_planning_msg::msg::TaskAction>(
    "input/task_action", 0,
    std::bind(&PathFollowerNode::task_action_subscriber_callback, this, std::placeholders::_1));

  pub_twist_ = create_publisher<geometry_msgs::msg::Twist>("output/cmd_vel", 0);
  pub_action_result =
    create_publisher<rw_planning_msg::msg::ActionResult>("output/action_result", 0);

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&PathFollowerNode::timer_callback, this));

  pose_status = not_found;
  path_status = not_found;
}

void PathFollowerNode::timer_callback()
{
  if (pose_status != ready || path_status != ready) {
    return;
  }

  bool flag1 = false;
  bool flag2 = false;

  bool is_end = false;
  if (target_pose_index >= global_path.poses.size()) {
    target_pose_index = global_path.poses.size() - 1;
    is_end = true;
  }

  const auto & target_pose = global_path.poses[target_pose_index].pose.position;
  const auto & target_orientation = global_path.poses[target_pose_index].pose.orientation;
  const auto & current_position = current_pose.pose.position;
  const auto & current_orientation = current_pose.pose.orientation;

  double dx = target_pose.x - current_position.x;
  double dy = target_pose.y - current_position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  const double threshold = 0.05;

  double vec_x = 0.0;
  double vec_y = 0.0;

  if (distance >= threshold) {
    double rad = std::atan2(dy, dx);
    vec_x = std::cos(rad);
    vec_y = std::sin(rad);
  } else {
    flag1 = true;
    if (is_end) {
      vec_x = 0.0;
      vec_y = 0.0;
    }
  }

  // RCLCPP_INFO_STREAM(get_logger(), "x: " << target_pose.x << " y: " << target_pose.y << " err: " << distance);

  const auto rqy_current = rw_common_util::geometry::quat_to_euler(current_orientation);
  const auto rqy_goal = rw_common_util::geometry::quat_to_euler(target_orientation);

  // RCLCPP_INFO_STREAM(get_logger(), "c: " << rqy_current.yaw * 57.295 << " g: " << rqy_goal.yaw * 57.295 << " err: " << (rqy_current.yaw - rqy_goal.yaw) * 57.295);
  double vec_z = 0.0;

  double err = std::abs(rqy_current.yaw - rqy_goal.yaw);
  if (err * 57.295 < 5) {
    flag2 = true;
    err = 0.0;
  }

  if (flag1 && flag2) {
    target_pose_index++;
  }

  if (flag1 && flag2 && is_end) {
    auto action_result_msg = rw_planning_msg::msg::ActionResult();
    action_result_msg.status.code = rw_common_msgs::msg::Status::SUCCESS;
    action_result_msg.status.success = true;
    action_result_msg.result_pose = current_pose;
    action_result_msg.task_id = task_id;
    pub_action_result->publish(action_result_msg);

    std::ofstream log_file;
    log_file.open("/tmp/rw.log", std::ios::app);
    log_file << "result , " << std::to_string(current_position.x) << ","
             << std::to_string(current_position.y) << ","
             << std::to_string(rqy_current.yaw * 57.295779513) << std::endl;
    log_file.close();

    pose_status = not_found;
    path_status = not_found;
  }

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = vec_x * -0.2;
  twist_msg.linear.y = vec_y * -0.2;
  twist_msg.angular.z = err * 1.5;

  if (std::signbit(twist_msg.linear.x) != std::signbit(dx)) {
    twist_msg.linear.x *= -1.0;
  }
  if (std::signbit(twist_msg.linear.y) != std::signbit(dy)) {
    twist_msg.linear.y *= -1.0;
  }
  if (std::signbit(twist_msg.angular.z) != std::signbit(rqy_current.yaw - rqy_goal.yaw)) {
    twist_msg.angular.z *= -1.0;
  }

  pub_twist_->publish(twist_msg);
}

void PathFollowerNode::current_pose_subscriber_callback(
  const geometry_msgs::msg::PoseStamped & sub_msg_pose)
{
  current_pose = sub_msg_pose;
  pose_status = ready;
}

void PathFollowerNode::nav_path_subscriber_callback(const nav_msgs::msg::Path & sub_msg_path)
{
  global_path = sub_msg_path;
  const auto path_size = global_path.poses.size();

  if (path_size == 0) {
    path_status = not_found;
    return;
  } else {
    path_status = ready;
    RCLCPP_INFO_STREAM(get_logger(), "Get global path");
  }

  std::vector<std::pair<float, size_t>> distances(path_size);
  for (size_t i = 0; i < path_size; i++) {
    const auto dx = global_path.poses[i].pose.position.x - current_pose.pose.position.x;
    const auto dy = global_path.poses[i].pose.position.y - current_pose.pose.position.y;
    distances[i] = {dx * dx + dy * dy, i};
  }
  auto min_iter = std::min_element(distances.begin(), distances.end());
  start_pose_index = min_iter->second;
  start_pose = global_path.poses[start_pose_index];
  goal_pose = global_path.poses.back();

  RCLCPP_INFO_STREAM(
    get_logger(), "Control NOW:" << start_pose.pose.position.x << "," << start_pose.pose.position.y
                                 << "," << start_pose_index);

  target_pose_index = start_pose_index;
}

void PathFollowerNode::task_action_subscriber_callback(
  const rw_planning_msg::msg::TaskAction & action_msg)
{
  task_id = action_msg.id;
}
}  // namespace path_follower

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_follower::PathFollowerNode)