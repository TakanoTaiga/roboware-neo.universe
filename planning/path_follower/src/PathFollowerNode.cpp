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
        point_state_manager.paramset(0.2, 0.0872665);
        sub_current_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "input/current_pose", 0, std::bind(&PathFollowerNode::current_pose_subscriber_callback, this, std::placeholders::_1));
        sub_nav_path_ = create_subscription<nav_msgs::msg::Path>(
            "input/nav_path", 0, std::bind(&PathFollowerNode::nav_path_subscriber_callback, this, std::placeholders::_1));
        sub_task_action_ = create_subscription<rw_planning_msg::msg::TaskAction>(
            "input/task_action", 0, std::bind(&PathFollowerNode::task_action_subscriber_callback, this, std::placeholders::_1));

        pub_action_result = create_publisher<rw_planning_msg::msg::ActionResult>(
            "output/action_result", 0);
        pub_cmd_pose = create_publisher<geometry_msgs::msg::Pose>(
            "output/cmd_pose", 0);

        pub_debug_target_pose = create_publisher<geometry_msgs::msg::PoseStamped>(
            "debug/target_pose", 0);

        position_tolerance = declare_parameter<double>("position_tolerance" , 0.01);
        angle_tolerance = declare_parameter<double>("angle_tolerance" , 1.0);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&PathFollowerNode::timer_callback, this));
    }

    void PathFollowerNode::timer_callback()
    {
        if (!point_state_manager.isReady()) {
            return;
        }

        auto& current_pose = point_state_manager.current_pose;
        auto& current_path = point_state_manager.current_path;

        // PurePursuit 
        if (current_path.poses.empty()) {
            // RCLCPP_WARN_STREAM(get_logger(), "current_path is empty");
            return;
        }

        std::vector<std::pair<double, size_t>> distances;
        std::vector<std::pair<double, size_t>> near_distances;
        distances.reserve(current_path.poses.size());
        near_distances.reserve(current_path.poses.size());

        for (size_t i = 0; i < current_path.poses.size(); ++i) {
            const auto norm = point_state_manager.norm2(current_path.poses[i].pose.position, current_pose.pose.position);
            if (norm >= point_state_manager.param_pos_err_max) {
                distances.emplace_back(norm, i);
            }else{
                near_distances.emplace_back(norm, i);
            }
        }

        if (distances.empty()) {
            distances.emplace_back(0.0, current_path.poses.size() - 1);
        }
        const auto target_iter = std::min_element(distances.begin(), distances.end());

        auto erase_end = current_path.poses.begin() + target_iter->second;
        current_path.poses.erase(current_path.poses.begin(), erase_end);

        const auto& target_position = current_path.poses.front().pose.position;
        const auto& current_position = current_pose.pose.position;

        using rw_common_util::geometry::operator-;
        const auto delta_position = target_position - current_position;

        double angle = std::atan2(delta_position.y, delta_position.x);
        const double speed = target_position.z + 0.15;
        
        auto linear_x = -1.0 * speed * std::cos(angle);
        auto linear_y = -1.0 * speed * std::sin(angle);

        const auto is_ok_pos_x = std::abs(delta_position.x) < position_tolerance;
        if(is_ok_pos_x){
            linear_x = 0.0;
        }

        const auto is_ok_pos_y = std::abs(delta_position.y) < position_tolerance;
        if(is_ok_pos_y){
            linear_y = 0.0;
        }

        if (std::signbit(linear_x) != std::signbit(delta_position.x)) {
            linear_x *= -1.0;
        }
        if (std::signbit(linear_y) != std::signbit(delta_position.y)) {
            linear_y *= -1.0;
        }

        // PurePursuit end.

        auto cmd_pose = geometry_msgs::msg::Pose();
        cmd_pose.position.x = linear_x;
        cmd_pose.position.y = linear_y;
        cmd_pose.orientation = current_path.poses.front().pose.orientation;
        pub_cmd_pose->publish(cmd_pose);

        const auto current_yaw = rw_common_util::geometry::quat_to_euler(current_pose.pose.orientation).yaw;

        double err = point_state_manager.norm2(current_pose.pose.orientation, current_path.poses.front().pose.orientation); 
        const auto is_ok_angle = std::abs(err * 57.295) < angle_tolerance;

        if(is_ok_pos_x && is_ok_pos_y && is_ok_angle){
            auto action_result_msg = rw_planning_msg::msg::ActionResult();
            action_result_msg.status.code = rw_common_msgs::msg::Status::SUCCESS;
            action_result_msg.status.success = true;
            action_result_msg.result_pose = current_pose;
            action_result_msg.task_id = task_id;
            pub_action_result->publish(action_result_msg);

            std::ofstream log_file;
            log_file.open("/tmp/rw.log", std::ios::app);
            log_file << "result , " << std::to_string(current_position.x) << "," << std::to_string(current_position.y) << "," << std::to_string(current_yaw * 57.295779513) << std::endl;
            log_file.close();
            
            point_state_manager.setWait();
        }

        if(!current_path.poses.empty())
        {
            pub_debug_target_pose->publish(current_path.poses.front());
        }
    }


    void PathFollowerNode::current_pose_subscriber_callback(const geometry_msgs::msg::PoseStamped& sub_msg_pose)
    {
        point_state_manager.setCurrentPose(sub_msg_pose);
    }

    void PathFollowerNode::nav_path_subscriber_callback(const nav_msgs::msg::Path& sub_msg_path)
    {
        point_state_manager.setpath(sub_msg_path);
    }

    void PathFollowerNode::task_action_subscriber_callback(const rw_planning_msg::msg::TaskAction& action_msg)
    {
        task_id = action_msg.id;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_follower::PathFollowerNode)