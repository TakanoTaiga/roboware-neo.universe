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

      cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 379.79150390625, 0.0, 314.5777587890625,
                                                      0.0, 379.31451416015625, 246.184814453125,
                                                      0.0, 0.0, 1.0);

      cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -0.05570452660322189, 0.06647524237632751,
                                                    0.00021408198517747223, -0.00022482164786197245,
                                                    -0.020616639405488968);

      // マーカーの一辺の長さ（メートル）
      float markerLength = 0.1;  // 実際のマーカーのサイズに合わせて調整

      auto cv_image = cv_bridge::toCvShare(msg, msg->encoding);
      std::vector<int> markerIds;
      std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
      auto detectorParams = cv::aruco::DetectorParameters::create();
      auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
      cv::aruco::detectMarkers(cv_image->image, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

      std::vector<cv::Vec3d> rvecs, tvecs;
      if(markerIds.size() > 0){
        cv::aruco::drawDetectedMarkers(cv_image->image, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

        for(size_t i = 0; i < markerIds.size(); i++) {
          geometry_msgs::msg::TransformStamped transformStamped;

          transformStamped.header.stamp = now();
          transformStamped.header.frame_id = "camera";
          transformStamped.child_frame_id = "marker_" + std::to_string(markerIds[i]);
          transformStamped.transform.translation.x =  tvecs[i][2];
          transformStamped.transform.translation.y = -tvecs[i][0];
          transformStamped.transform.translation.z = -tvecs[i][1];

          tf2::Quaternion q;
          double r = rvecs[i][0], p = rvecs[i][1], y = rvecs[i][2];
          q.setRPY(r, p, y);
          transformStamped.transform.rotation.x = q.x();
          transformStamped.transform.rotation.y = q.y();
          transformStamped.transform.rotation.z = q.z();
          transformStamped.transform.rotation.w = q.w();


          tf_broadcaster_->sendTransform(transformStamped);
        }
      }

      auto ros_img_msg = sensor_msgs::msg::Image();
      cv_image->toImageMsg(ros_img_msg);
      pub_debug_image_->publish(ros_img_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detect_ar_marker::detectARMakerNode)