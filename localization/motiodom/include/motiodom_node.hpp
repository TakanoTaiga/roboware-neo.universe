#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <string>

#include "ekf6_utils.hpp"
#include "ekf9_utils.hpp"
#include "matrix_utils.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace motiodom
{
    class MotiOdom : public rclcpp::Node
    {
        public:
        explicit MotiOdom(const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());

        private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void magnet_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
        void axis6_callback();
        void axis9_callback();

        Vector3 remove_gravity(Vector3 linear_accel, Vector3 euler, float gravity);
        Vector3 noise_filter(Vector3 value, float alpha);

        float to_radian(float degree);
        Matrix3x3 rotation_from_euler(Vector3 euler);

        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr magnetic_field_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // ROS2 Parameters
        bool enable_magnet_, imu_flag_, mag_flag_, enable_position_;
        std::string frame_id_, child_id_;

        std::shared_ptr<Axis6EKF> ekf6_;
        std::shared_ptr<Axis9EKF> ekf9_;
        Vector3 prev_input_;
        Vector3 prev_output_;

        Vector3 prev_accel_;
        Vector3 prev_vel;

        sensor_msgs::msg::Imu::SharedPtr get_imu_;
        geometry_msgs::msg::Vector3::SharedPtr get_magnet_;
    };
}

#endif