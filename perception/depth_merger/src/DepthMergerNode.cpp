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

#include "depth_merge_node/DepthMergerNode.hpp"

namespace depth_merge_node
{
    DepthMergerNode::DepthMergerNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("depth_merger_node", node_option), 
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) 
    {
        sub_2d_bbox_ = create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
            "input/bbox", 0, std::bind(&DepthMergerNode::bbox_subscriber_callback, this, std::placeholders::_1));
        sub_depth_image_ = create_subscription<sensor_msgs::msg::Image>(
            "input/depth_raw", 0, std::bind(&DepthMergerNode::image_subscriber_callback, this, std::placeholders::_1));
        sub_cam_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "input/camera_info", 0, std::bind(&DepthMergerNode::cam_info_subscriber_callback, this, std::placeholders::_1));

        pub_poses_ = create_publisher<rw_common_msgs::msg::TransformArray>(
            "output/poses", 0);

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        param_base_frame = declare_parameter<std::string>("base_frame" , "base_link");
    }

    void DepthMergerNode::image_subscriber_callback(const sensor_msgs::msg::Image& msg)
    {
        auto convertTimestampToDouble = [](builtin_interfaces::msg::Time time) {
            auto sec = time.sec; 
            auto nanosec = time.nanosec;
            constexpr double NANOSECONDS_IN_SECOND = 1000000000.0;
            return static_cast<double>(sec) + static_cast<double>(nanosec) / NANOSECONDS_IN_SECOND;
        };

        image_cache[util::get_key(msg.header)] = std::make_pair(convertTimestampToDouble(get_clock()->now()), msg);

        std::vector<util::key_time> key_list;
        for(const auto& [key, image_and_time] : image_cache)
        {
            const auto& [add_time, image] = image_and_time;
            const auto now_time = convertTimestampToDouble(get_clock()->now());
            const auto dt = std::abs(now_time - add_time);
            if(dt > 1.0){
                key_list.push_back(key);
            }
        }

        for(const auto& key : key_list)
        {
            image_cache.erase(key);
        }
    }

    void DepthMergerNode::cam_info_subscriber_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx = msg->k[0];
        fy = msg->k[4];
        cx = msg->k[2];
        cy = msg->k[5];
    }

    void DepthMergerNode::bbox_subscriber_callback(const bboxes_ex_msgs::msg::BoundingBoxes msg)
    {
        if(image_cache.size() == 0){ return; }

        if(!rw_common_util::contains(image_cache, util::get_key(msg.image_header))){ return; }

        const auto& [_, image] = image_cache[util::get_key(msg.image_header)];
        auto cv_image = cv_bridge::toCvShare(sensor_msgs::msg::Image::ConstSharedPtr(new sensor_msgs::msg::Image(image)), image.encoding);

        auto poses_rosmsg = rw_common_msgs::msg::TransformArray();
        for (const auto& bbox : msg.bounding_boxes) {
            uint64_t depth_sum = 0;
            uint32_t count = 0; 
            uint32_t err_count = 0;

            for (uint16_t y = bbox.ymin; y <= bbox.ymax; ++y) {
                for (uint16_t x = bbox.xmin; x <= bbox.xmax; ++x) {
                    const auto depth_mm = cv_image->image.at<uint16_t>(y, x);
                    depth_sum += depth_mm;
                    ++count;
                }
            }

            if (count == 0) {
                continue;
            }

            const auto depth_avg_m = static_cast<double>(depth_sum) / static_cast<double>(count) / 1000.0f;
            const auto bbox_size_x = bbox.xmax - bbox.xmin;
            const auto bbox_size_y = bbox.ymax - bbox.ymin;
            const auto position_x = ((bbox.xmin + bbox_size_x / 2) - cx) * depth_avg_m / fx;
            const auto position_y = ((bbox.ymin + bbox_size_y / 2) - cy) * depth_avg_m / fy;
            const auto position_z = depth_avg_m;

            geometry_msgs::msg::TransformStamped transform;
            transform.header = image.header;
            transform.child_frame_id = bbox.class_id + "(" + std::to_string(bbox.id) + ")";
            transform.transform.translation.x = position_x;
            transform.transform.translation.y = position_y;
            transform.transform.translation.z = position_z;

            try {
                auto map_transform = tf_buffer_.lookupTransform(param_base_frame, transform.header.frame_id, transform.header.stamp, rclcpp::Duration::from_seconds(0.1)); //tiemr
                tf2::doTransform(transform, transform, map_transform);
                tf_broadcaster_->sendTransform(transform);
                poses_rosmsg.transforms.push_back(transform);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
                continue;
            }
        }

        pub_poses_->publish(poses_rosmsg);

        image_cache.erase(util::get_key(msg.image_header));
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depth_merge_node::DepthMergerNode)
