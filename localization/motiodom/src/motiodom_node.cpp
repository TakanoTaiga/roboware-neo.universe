#include "motiodom_node.hpp"

namespace motiodom
{
    MotiOdom::MotiOdom(const rclcpp::NodeOptions & node_options): rclcpp::Node("motiodom_node", node_options)
    {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            0,
            std::bind(&MotiOdom::imu_callback, this, _1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        this->declare_parameter("enable_magnet", false);
        this->get_parameter("enable_magnet", enable_magnet_);

        this->declare_parameter("enable_position", false);
        this->get_parameter("enable_position", enable_position_);

        this->declare_parameter("frame_id", "odom");
        this->get_parameter("frame_id", frame_id_);

        this->declare_parameter("child_frame_id", "imu");
        this->get_parameter("child_frame_id", child_id_);

        imu_flag_ = false;

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
                get_imu_->linear_acceleration.y,
                get_imu_->linear_acceleration.z);

            auto angular_velocity = Vector3(
                to_radian<float>(get_imu_->angular_velocity.x),
                to_radian<float>(get_imu_->angular_velocity.y),
                to_radian<float>(get_imu_->angular_velocity.z));

            auto input_matrix = Vector3(
                angular_velocity.x*0.01,
                angular_velocity.y*0.01,
                angular_velocity.z*0.01);


            auto jacob = calc_jacob(input_matrix, ekf6_.est);

            ekf6_.est = predict_x(input_matrix, ekf6_.est);

            ekf6_.cov = predict_cov(jacob, ekf6_.cov, ekf6_.est_noise);

            auto z = obs_model_6(linear_accel);

            auto residual = update_residual(z, ekf6_.est);

            auto s = update_s(ekf6_.cov, ekf6_.obs_noise);

            ekf6_.k_gain = update_kalman_gain(s, ekf6_.cov);

            ekf6_.est = update_x(ekf6_.est, ekf6_.k_gain, residual);

            ekf6_.cov = update_cov(ekf6_.k_gain, ekf6_.cov);

            auto estimated = Vector3(ekf6_.est.x, ekf6_.est.y, ekf6_.est.z);

            geometry_msgs::msg::TransformStamped t;

            t.header.frame_id = frame_id_;
            t.header.stamp = this->get_clock()->now();
            t.child_frame_id = child_id_;

            tf2::Quaternion q;
            q.setRPY(estimated.x /2.0, estimated.y/2.0, estimated.z/2.0);
            t.transform.rotation.w = q.w();
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();

            tf_broadcaster_->sendTransform(t);
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
                to_radian<float>(get_imu_->angular_velocity.x),
                to_radian<float>(get_imu_->angular_velocity.y),
                to_radian<float>(get_imu_->angular_velocity.z));

            auto mag = Vector3(
                get_magnet_->x,
                get_magnet_->y,
                get_magnet_->z);

            predict_x(ekf9_, angular_velocity);

            auto obs = observation_model(linear_accel, mag);

            auto a = estimation_jacob(ekf9_, angular_velocity);

            auto c = Matrix3x3(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0);

            auto pre_cov = estimation_cov(ekf9_, a);
            auto obs_cov = observation_cov(ekf9_, pre_cov, c);

            kalman_gain(ekf9_, pre_cov, c, obs_cov);

            update_x(ekf9_, obs);
            update_cov(ekf9_, pre_cov);

            auto input_matrix = Vector3(
                angular_velocity.x*0.01,
                angular_velocity.y*0.01,
                angular_velocity.z*0.01);


            auto jacob = calc_jacob(input_matrix, ekf6_.est);

            ekf6_.est = predict_x(input_matrix, ekf6_.est);

            ekf6_.cov = predict_cov(jacob, ekf6_.cov, ekf6_.est_noise);

            auto z = obs_model_6(linear_accel);

            auto residual = update_residual(z, ekf6_.est);

            auto s = update_s(ekf6_.cov, ekf6_.obs_noise);

            ekf6_.k_gain = update_kalman_gain(s, ekf6_.cov);

            ekf6_.est = update_x(ekf6_.est, ekf6_.k_gain, residual);

            ekf6_.cov = update_cov(ekf6_.k_gain, ekf6_.cov);

            auto estimated = Vector3(ekf6_.est.x, ekf9_.est.y, ekf6_.est.z/2.0);

            auto g_removed = remove_gravity(linear_accel, estimated, 0.981);

            geometry_msgs::msg::TransformStamped t;

            t.header.frame_id = frame_id_;
            t.header.stamp = this->get_clock()->now();
            t.child_frame_id = child_id_;

            tf2::Quaternion q;
            q.setRPY(-estimated.x, -estimated.y, estimated.z);

            t.transform.rotation.w = q.w();
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();

            auto now_vel = Vector3(
                prev_vel.x + (g_removed.x+prev_accel_.x)*0.01 *0.5,
                prev_vel.y + (g_removed.y+prev_accel_.y)*0.01 *0.5,
                prev_vel.z + (g_removed.z+prev_accel_.z)*0.01 *0.5
            );

            if(enable_position_)
            {
                t.transform.translation.x -= noise_filter((now_vel.x+prev_vel.x)*0.01*0.5, 0.1);
                t.transform.translation.y -= noise_filter((now_vel.y+prev_vel.y)*0.01*0.5, 0.1);
                // t.transform.translation.z = (now_vel.z+prev_vel.z)*0.01*0.5;
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

    float MotiOdom::noise_filter(float value, float alpha)
    {
        if(abs(value) > alpha)
        {
            return value*1000.0;
        }
        else
        {
            return 0.0;
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motiodom::MotiOdom)
