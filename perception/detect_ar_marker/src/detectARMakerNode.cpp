// Copyright 2023 Hakoroboken
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


#include "detect_ar_marker/detectARMakerNode.hpp"

namespace detect_ar_marker
{
    detectARMakerNode::detectARMakerNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("bbox_to_foxglove_node", node_option)
    {   
      sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        "/sensing/realsense/color/image_raw", 0, std::bind(&detectARMakerNode::subscriber_callback, this, std::placeholders::_1));

      pub_debug_image_ = create_publisher<sensor_msgs::msg::Image>(
        "debug/image", 0);

      tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void detectARMakerNode::subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg){
      auto cv_image = cv_bridge::toCvShare(msg, msg->encoding);
      std::vector<int> markerIds;
      std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
      auto detectorParams = cv::aruco::DetectorParameters::create();
      auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
      cv::aruco::detectMarkers(cv_image->image, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

      if(markerIds.size() > 0){
				cv:: aruco::drawDetectedMarkers(cv_image->image, markerCorners, markerIds);  
			}

      auto ros_img_msg = sensor_msgs::msg::Image();
      cv_image->toImageMsg(ros_img_msg);
      pub_debug_image_->publish(ros_img_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detect_ar_marker::detectARMakerNode)