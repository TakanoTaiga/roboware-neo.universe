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
    PathFollowerNode::PathFollowerNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("PathFollowerNode", node_option)
    {   
        sub_current_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "input/current_pose", 0, std::bind(&PathFollowerNode::current_pose_subscriber_callback, this, std::placeholders::_1));
        sub_nav_path_ = create_subscription<nav_msgs::msg::Path>(
            "input/nav_path", 0, std::bind(&PathFollowerNode::nav_path_subscriber_callback, this, std::placeholders::_1));
        sub_task_action_ = create_subscription<rw_planning_msg::msg::TaskAction>(
            "input/task_action", 0, std::bind(&PathFollowerNode::task_action_subscriber_callback, this, std::placeholders::_1));

        pub_twist_ = create_publisher<geometry_msgs::msg::Twist>(
            "output/cmd_vel", 0);
        pub_action_result = create_publisher<rw_planning_msg::msg::ActionResult>(
            "output/action_result", 0);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PathFollowerNode::timer_callback, this));

        pose_status = not_found;
        path_status = not_found;
        p_status = wip;
    }

    void PathFollowerNode::timer_callback()
    {
        if(pose_status != ready || path_status != ready){return;}

        if(target_pose_index == global_path.poses.size()){
            auto twist_msg = geometry_msgs::msg::Twist();
            pub_twist_->publish(twist_msg);
            path_status = not_found;

            auto action_result_msg = rw_planning_msg::msg::ActionResult();
            action_result_msg.status.code = rw_common_msgs::msg::Status::SUCCESS;
            action_result_msg.status.success = true;
            action_result_msg.result_pose = current_pose;
            action_result_msg.task_id = task_id;
            pub_action_result->publish(action_result_msg);
            return;
        }

        auto dx = global_path.poses[target_pose_index].pose.position.x - current_pose.pose.position.x;
        auto dy = global_path.poses[target_pose_index].pose.position.y - current_pose.pose.position.y;
        auto target_distance = std::sqrt(dx * dx + dy * dy);
        if(target_distance < 0.1){
            target_pose_index++;
            dx = global_path.poses[target_pose_index].pose.position.x - current_pose.pose.position.x;
            dy = global_path.poses[target_pose_index].pose.position.y - current_pose.pose.position.y;
            target_distance = std::sqrt(dx * dx + dy * dy);
        }

        const auto rad = std::atan(dy / dx);
        const auto vec_x = std::cos(rad);
        const auto vec_y = std::sin(rad);

        tf2::Quaternion quat_current, quat_goal;
        tf2::fromMsg(current_pose.pose.orientation, quat_current);
        tf2::fromMsg(global_path.poses[target_pose_index].pose.orientation, quat_goal);

        tf2::Matrix3x3 mat_current(quat_current), mat_goal(quat_goal);
        double roll_current, pitch_current, yaw_current;
        double roll_goal, pitch_goal, yaw_goal;
        mat_current.getRPY(roll_current, pitch_current, yaw_current);
        mat_goal.getRPY(roll_goal, pitch_goal, yaw_goal);

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = vec_x * -0.2;
        twist_msg.linear.y = vec_y * -0.2;
        twist_msg.angular.z = (yaw_current - yaw_goal) * 1.0;

        if(std::signbit(twist_msg.linear.x) != std::signbit(dx)){twist_msg.linear.x *= -1.0;}
        if(std::signbit(twist_msg.linear.y) != std::signbit(dy)){twist_msg.linear.y *= -1.0;}

        RCLCPP_INFO_STREAM(get_logger(),  "Control NOW:" << target_distance << "," <<  target_pose_index << "," << global_path.poses.size());


        if(p_status == wip){
            pub_twist_->publish(twist_msg);
        }

    }

    void PathFollowerNode::current_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& sub_msg_pose)
    {
        current_pose = sub_msg_pose;
        pose_status = ready;
    }

    void PathFollowerNode::nav_path_subscriber_callback(const nav_msgs::msg::Path& sub_msg_path)
    {
        global_path = sub_msg_path;
        const auto path_size = global_path.poses.size();

        if(path_size == 0){
            path_status = not_found;
            return;
        }else{
            path_status = ready;
            RCLCPP_INFO_STREAM(get_logger(),  "Get global path");
        }

        std::vector<std::pair<float, size_t>> distances(path_size);
        for(size_t i = 0; i < path_size; i++ ){
            const auto dx = global_path.poses[i].pose.position.x - current_pose.pose.position.x;
            const auto dy = global_path.poses[i].pose.position.y - current_pose.pose.position.y;
            distances[i] = {dx * dx + dy * dy, i};
        }
        auto min_iter = std::min_element(distances.begin(), distances.end());
        start_pose_index = min_iter->second;
        start_pose = global_path.poses[start_pose_index];
        goal_pose = global_path.poses.back();
            
        RCLCPP_INFO_STREAM(get_logger(),  "Control NOW:" << start_pose.pose.position.x << "," <<  start_pose.pose.position.y << "," << start_pose_index);

        target_pose_index = start_pose_index;
    }

    void PathFollowerNode::task_action_subscriber_callback(const rw_planning_msg::msg::TaskAction& action_msg)
    {
        task_id = action_msg.id;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_follower::PathFollowerNode)