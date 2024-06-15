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

#ifndef MCU_ROS_DRIVER_NODE_HPP_
#define MCU_ROS_DRIVER_NODE_HPP_

#include <sstream>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rw_common_msgs/msg/udp_packet_binary.hpp>
#include <rw_common_msgs/msg/mcu_device_lists.hpp>

#include "mcu_ros_driver/mcu_util.hpp"

namespace mcu_ros_driver
{
    class mcu_ros_driver_node : public rclcpp::Node
    {
    public:
        explicit mcu_ros_driver_node(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Publisher<rw_common_msgs::msg::UDPPacketBinary>::SharedPtr pub_udp_packet_;
        rclcpp::Publisher<rw_common_msgs::msg::MCUDeviceLists>::SharedPtr pub_device_list_;
        rclcpp::Subscription<rw_common_msgs::msg::UDPPacketBinary>::SharedPtr sub_udp_packet;
        rclcpp::Subscription<rw_common_msgs::msg::MCUDeviceLists>::SharedPtr sub_mcu_device;
        rclcpp::TimerBase::SharedPtr mcu_list_timer_;

        void udp_subscriber_callback(const rw_common_msgs::msg::UDPPacketBinary& msg);
        void mcu_subscriber_callback(const rw_common_msgs::msg::MCUDeviceLists& msg);
        void mcu_list_pub_timer_callback();

        std::map<std::string, std::string> device_list;

    };
} // namespace mcu_ros_driver

#endif