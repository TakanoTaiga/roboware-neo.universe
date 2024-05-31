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

#include "detect_ar_marker/detectARMarkerNode.hpp"

namespace detect_ar_marker
{
detectARMarkerNode::detectARMarkerNode(const rclcpp::NodeOptions & node_option)
: rclcpp::Node("detect_ar_marker_node", node_option)
{
  sub_image_ = create_subscription<sensor_msgs::msg::Image>(
    "input/raw_image", 0,
    std::bind(&detectARMarkerNode::image_subscriber_callback, this, std::placeholders::_1));

  sub_cam_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "input/camera_info", 0,
    std::bind(&detectARMarkerNode::cam_info_subscriber_callback, this, std::placeholders::_1));

  pub_debug_image_ = create_publisher<sensor_msgs::msg::Image>("debug/image", 0);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  markerLength = declare_parameter<double>("marker_length", 0.1);

  cameraMatrix = (cv::Mat_<double>(3, 3) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  distCoeffs = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
}

void detectARMarkerNode::image_subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto cv_image = cv_bridge::toCvShare(msg, msg->encoding);
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  auto detectorParams = cv::aruco::DetectorParameters::create();
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  cv::aruco::detectMarkers(
    cv_image->image, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

  std::vector<cv::Vec3d> rvecs, tvecs;
  if (markerIds.size() > 0) {
    cv::aruco::drawDetectedMarkers(cv_image->image, markerCorners, markerIds);
    cv::aruco::estimatePoseSingleMarkers(
      markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

    for (size_t i = 0; i < markerIds.size(); i++) {
      geometry_msgs::msg::TransformStamped transformStamped;

      transformStamped.header.stamp = now();
      transformStamped.header.frame_id = msg->header.frame_id;
      transformStamped.child_frame_id = "marker_" + std::to_string(markerIds[i]);
      transformStamped.transform.translation.x = tvecs[i][0];
      transformStamped.transform.translation.y = tvecs[i][1];
      transformStamped.transform.translation.z = tvecs[i][2];

      // rvecsをロドリゲスの回転ベクトルから回転行列に変換
      cv::Mat rotMat;
      cv::Rodrigues(rvecs[i], rotMat);

      // 回転行列からtf2::Matrix3x3を作成
      tf2::Matrix3x3 tfRotMat(
        rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
        rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
        rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));

      // tf2::Quaternionに変換
      tf2::Quaternion q;
      tfRotMat.getRotation(q);

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

void detectARMarkerNode::cam_info_subscriber_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  msg->k[0];
  cameraMatrix =
    (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5],
     msg->k[6], msg->k[7], msg->k[8]);

  distCoeffs = (cv::Mat_<double>(5, 1) << msg->d[0], msg->d[1], msg->d[2], msg->d[3], msg->d[4]);
}

}  // namespace detect_ar_marker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detect_ar_marker::detectARMarkerNode)