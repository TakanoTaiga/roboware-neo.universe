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

#include "mcu_ros_driver/mcu_ros_driver_node.hpp"

namespace mcu_ros_driver
{
    mcu_ros_driver_node::mcu_ros_driver_node(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("mcu_ros_driver_node", node_option)
    {
        pub_udp_packet_ = create_publisher<rw_common_msgs::msg::UDPPacketBinary>(
            "output/udp_packet", 0);
        pub_device_list_ = create_publisher<rw_common_msgs::msg::MCUDeviceLists>(
            "output/device_list", 0);
        sub_udp_packet = create_subscription<rw_common_msgs::msg::UDPPacketBinary>(
            "input/udp_packet", 0, std::bind(&mcu_ros_driver_node::udp_subscriber_callback, this, std::placeholders::_1));
        sub_mcu_device = create_subscription<rw_common_msgs::msg::MCUDeviceLists>(
            "input/mcu_order", 0, std::bind(&mcu_ros_driver_node::mcu_subscriber_callback, this, std::placeholders::_1));

        mcu_list_timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&mcu_ros_driver_node::mcu_list_pub_timer_callback, this));

    }

    void mcu_ros_driver_node::udp_subscriber_callback(const rw_common_msgs::msg::UDPPacketBinary& msg)
    {
        const auto [addr, name] = mcu_util::mcu_info_parser(msg.data);
        device_list[name] = addr;
    }

    void mcu_ros_driver_node::mcu_subscriber_callback(const rw_common_msgs::msg::MCUDeviceLists& msg)
    {
        for(const auto& device : msg.devices)
        {
            auto pub_msg = rw_common_msgs::msg::UDPPacketBinary();
            pub_msg.address = device_list[device.device];
            pub_msg.data = std::vector<uint8_t>(device.data.begin(),device.data.end());
            pub_msg.port = 64202;
            pub_udp_packet_->publish(pub_msg);
        }
    }

    void mcu_ros_driver_node::mcu_list_pub_timer_callback()
    {
        auto pubmsg = rw_common_msgs::msg::MCUDeviceLists();
        for(const auto& [name, _] : device_list)
        {
            auto device_msg = rw_common_msgs::msg::MCUDevice();
            device_msg.device = name;
            pubmsg.devices.push_back(device_msg);
        }
        pub_device_list_->publish(pubmsg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mcu_ros_driver::mcu_ros_driver_node)
