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

#ifndef LOCALIZATION_DEBUG_NODE_HPP
#define LOCALIZATION_DEBUG_NODE_HPP

#include <vector>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace localization_debug
{
    class localizationDebugNode : public rclcpp::Node
    {
    public:
        explicit localizationDebugNode(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_maker_;
        rclcpp::TimerBase::SharedPtr pub_timer_;

        void timer_callback();

        visualization_msgs::msg::Marker marker_msg;
    };
} // namespace localization_debug

#endif