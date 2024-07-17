#include "motiodom_node.hpp"

namespace motiodom
{
    MotiOdom::MotiOdom(const rclcpp::NodeOptions & node_options): rclcpp::Node("motiodom_node", node_options)
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            qos_settings,
            std::bind(&MotiOdom::imu_callback, this, _1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_stamped", 0);

        this->declare_parameter("enable_magnet", false);
        this->get_parameter("enable_magnet", enable_magnet_);

        this->declare_parameter("enable_position", false);
        this->get_parameter("enable_position", enable_position_);

        this->declare_parameter("frame_id", "odom");
        this->get_parameter("frame_id", frame_id_);

        this->declare_parameter("child_frame_id", "imu");
        this->get_parameter("child_frame_id", child_id_);

        imu_flag_ = false;

        ekf6_ = std::make_shared<Axis6EKF>();
        ekf9_ = std::make_shared<Axis9EKF>();
        prev_input_ = Vector3();
        prev_output_ = Vector3();
        RCLCPP_INFO(this->get_logger(), "Initialized EKF");

        if(enable_magnet_)
        {
            magnetic_field_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/magnet",
                0,
                std::bind(&MotiOdom::magnet_callback, this, _1));
            mag_flag_ = false;

            timer_ = this->create_wall_timer(10ms, std::bind(&MotiOdom::axis9_callback, this));
        }
        else
        {
            timer_ = this->create_wall_timer(10ms, std::bind(&MotiOdom::axis6_callback, this));
        }

        RCLCPP_INFO(this->get_logger(), "Start MotiOdom delta_time: 10ms");

        
    }

    void MotiOdom::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_flag_ = true;
        get_imu_ = msg;
    }

    void MotiOdom::magnet_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        mag_flag_ = true;
        get_magnet_ = msg;
    }

    void MotiOdom::axis6_callback()
    {
        if(imu_flag_)
        {
            
            auto linear_accel = Vector3(
                get_imu_->linear_acceleration.x,
                get_imu_->linear_acceleration.z,
                get_imu_->linear_acceleration.y);

            auto angular_velocity = Vector3(
                get_imu_->angular_velocity.x,
                get_imu_->angular_velocity.z,
                get_imu_->angular_velocity.y);

            auto input_matrix = Vector3(
                angular_velocity.x*0.01,
                angular_velocity.y*0.01,
                angular_velocity.z*0.01);


            auto estimated = ekf6_->run_ekf6(input_matrix, linear_accel);

            geometry_msgs::msg::TransformStamped t;
            geometry_msgs::msg::PoseStamped pose;

            t.header.frame_id = frame_id_;
            t.header.stamp = this->get_clock()->now();
            t.child_frame_id = child_id_;

            tf2::Quaternion q;
            q.setRPY(estimated.x, estimated.y, estimated.z);
            t.transform.rotation.w = q.w();
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();

            tf_broadcaster_->sendTransform(t);
            pose.pose.orientation = t.transform.rotation;
            pose_publisher_->publish(pose);
        }
    }

    void MotiOdom::axis9_callback()
    {
        if(imu_flag_ && mag_flag_)
        {
            auto linear_accel = Vector3(
                get_imu_->linear_acceleration.x,
                get_imu_->linear_acceleration.y,
                get_imu_->linear_acceleration.z);

            auto angular_velocity = Vector3(
                to_radian(get_imu_->angular_velocity.x),
                to_radian(get_imu_->angular_velocity.y),
                to_radian(get_imu_->angular_velocity.z));

            auto mag = Vector3(
                get_magnet_->x,
                get_magnet_->y,
                get_magnet_->z);

            auto input_matrix = Vector3(
                angular_velocity.x*0.01,
                angular_velocity.y*0.01,
                angular_velocity.z*0.01);

            auto est6 = ekf6_->run_ekf6(input_matrix, linear_accel);
            auto est9 =ekf9_->run_ekf9(angular_velocity, linear_accel, mag);

            
            auto estimated = Vector3(est6.x*2.0, est9.y/2.0, est6.z);

            auto g_removed = remove_gravity(linear_accel, estimated, 0.981);

            geometry_msgs::msg::TransformStamped t;

            t.header.frame_id = frame_id_;
            t.header.stamp = this->get_clock()->now();
            t.child_frame_id = child_id_;

            tf2::Quaternion q;
            q.setRPY(estimated.x, estimated.y, estimated.z);

            t.transform.rotation.w = q.w();
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();

            auto now_vel = Vector3(
                (g_removed.x+prev_accel_.x)*0.01 *0.5,
                (g_removed.y+prev_accel_.y)*0.01 *0.5,
                (g_removed.z+prev_accel_.z)*0.01 *0.5
            );

            if(enable_position_)
            {
                Vector3 calc_pos;
                calc_pos.x = (now_vel.x + prev_vel.x)*0.01*0.5;
                calc_pos.y = (now_vel.y + prev_vel.y)*0.01*0.5;
                calc_pos.z = (now_vel.z + prev_vel.z)*0.01*0.5;

                auto filtered = noise_filter(calc_pos, 0.3);
                t.transform.translation.x += filtered.x;
                t.transform.translation.y += filtered.y;
                // t.transform.translation.z = filtered.z;
            }

            tf_broadcaster_->sendTransform(t);

            prev_accel_ = g_removed;
            prev_vel = now_vel;
        }
    }

    Vector3 MotiOdom::remove_gravity(Vector3 linear_accel, Vector3 euler, float gravity)
    {
        auto rm = rotation_from_euler(euler);

        auto g = Vector3(0.0, 0.0, gravity);

        auto removed = multiply(rm, g);

        auto g_removed = Vector3(
            removed.x + linear_accel.x,
            removed.y + linear_accel.y,
            removed.z - linear_accel.z);

        return g_removed;
    }

    Vector3 MotiOdom::noise_filter(Vector3 value, float alpha)
    {
        Vector3 output;
        output.x = alpha * (prev_output_.x + value.x - prev_output_.x);
        output.y = alpha * (prev_output_.y + value.y - prev_output_.y);
        output.z = alpha * (prev_output_.z + value.z - prev_output_.z);

        prev_output_ = output;
        prev_input_ = value;

        return output;

    }

    float MotiOdom::to_radian(float degree)
    {
        auto pi = acos(-1.0);


        return (degree*pi)/180.0;
    }

    Matrix3x3 MotiOdom::rotation_from_euler(Vector3 euler)
    {
        auto sin_x = sin(euler.x);
        auto sin_y = sin(euler.y);
        auto sin_z = sin(euler.z);
        auto cos_x = cos(euler.x);
        auto cos_y = cos(euler.y);
        auto cos_z = cos(euler.z);
        return Matrix3x3(
            cos_y*cos_z, -1.0*cos_y*sin_z, sin_y,
            sin_x*sin_y*cos_z+cos_x*sin_z, -1.0*sin_x*sin_y*sin_z + cos_x*cos_z, -1.0*sin_x*cos_y,
            -1.0*cos_x*sin_y*cos_z + sin_x*sin_z, cos_x*sin_y*sin_z + sin_x*cos_z, cos_x*cos_y
        );
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motiodom::MotiOdom)