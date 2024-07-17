#include "ekf9_utils.hpp"

namespace motiodom
{
    Axis9EKF::Axis9EKF():
        est(Vector3(0.0, 0.0, 0.0)), 
        cov(Matrix3x3(
            0.0174*0.01*0.01, 0.0, 0.0,
            0.0, 0.0174*0.01*0.01, 0.0,
            0.0, 0.0, 0.0174*0.01*0.01)),
        est_noise(Matrix3x3(
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01)),
        obs_noise(Matrix3x3(
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        )),
        k_gain(Matrix3x3(
            0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0)){}

    Vector3 Axis9EKF::run_ekf9(Vector3 angular_velocity, Vector3 linear_accel, Vector3 magnet)
    {
        est = pre_x(est, angular_velocity);

        auto obs = obs_model(linear_accel, magnet);

        auto est_jacob = esti_jacob(est, angular_velocity);

        auto identify = Matrix3x3(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);

        auto pre_cov = esti_cov(cov, est_jacob, est_noise);

        auto obs_cov = obse_cov(obs_noise, pre_cov, identify);

        k_gain = kalman_gain(pre_cov, identify, obs_cov);

        est = update__x(est, obs, k_gain);
        cov = update__cov(k_gain, pre_cov);

        return est;
    }

    Vector3 pre_x(Vector3 prev, Vector3 angular_velocity)
    {
        Vector3 est;
        est.x = prev.x + (angular_velocity.x + angular_velocity.y*tan(prev.y)*sin(prev.x) + angular_velocity.z*tan(prev.y)*cos(prev.x))*0.01;
        est.y = prev.y + (angular_velocity.y*cos(prev.x) - angular_velocity.z*sin(prev.x))*0.01;
        est.z = prev.z + (angular_velocity.y*(sin(prev.x)/cos(prev.y)) + angular_velocity.z*(cos(prev.x)/cos(prev.y)))*0.01;

        return est;
    }
    Vector3 obs_model(Vector3 linear_accel, Vector3 mag_)
    {
        Vector3 result(0.0, 0.0, 0.0);

        if(linear_accel.z == 0.0)
        {
            if(linear_accel.y > 0.0)
            {
                result.x = acos(-1.0) / 2.0;
            }
            else
            {
                result.x = -1.0*acos(-1.0) / 2.0;
            }
        }
        else
        {
            result.x = atan(linear_accel.y / linear_accel.z);
        }

        if(sqrt(linear_accel.y*linear_accel.y + linear_accel.z*linear_accel.z) == 0.0)
        {
            if(-1.0*linear_accel.x > 0.0)
            {
                result.y = acos(-1.0) / 2.0;
            }
            else
            {
                result.y = -1.0*acos(-1.0) / 2.0;
            }
        }
        else
        {
            result.y = (-1.0*linear_accel.x) / atan(sqrt(linear_accel.y*linear_accel.y+linear_accel.z*linear_accel.z));
        }

        float above = mag_.x*cos(result.y) + mag_.y*sin(result.y)*sin(result.x) + mag_.z*sin(result.y)*cos(result.x);
        float below = mag_.y*cos(result.x) - mag_.z*sin(result.x);

        result.z = atan(above / below);

        return result;
    }

    Matrix3x3 esti_jacob(Vector3 est, Vector3 angular_velocity)
    {
        return Matrix3x3(
            1.0 + (angular_velocity.x + angular_velocity.y*tan(est.y)*cos(est.x) - angular_velocity.z*tan(est.y*sin(est.x)))*0.01,
            (angular_velocity.y*(sin(est.x)/pow(cos(est.y), 2)) + angular_velocity.z*(cos(est.x)/pow(cos(est.y), 2)))*0.01,
            0.0,
            (-1.0*angular_velocity.y*sin(est.x) - angular_velocity.z*cos(est.x))*0.01,
            1.0,
            0.0,
            (angular_velocity.y*(cos(est.x)/cos(est.y)) - angular_velocity.z*(sin(est.x)/cos(est.y)))*0.01,
            (angular_velocity.y*sin(est.x)*(sin(est.y)/pow(cos(est.y), 2)) + angular_velocity.z*cos(est.x)*(sin(est.y)/pow(cos(est.y), 2)))*0.01,
            1.0
        );
    }

    Matrix3x3 esti_cov(Matrix3x3 cov, Matrix3x3 est_jacob, Matrix3x3 est_noise)
    {
        auto t_jacob = transpose_matrix(est_jacob);

        auto j_cov = multiply(est_jacob, cov);

        auto j_cov_t_j = multiply(j_cov, t_jacob);

        return add(j_cov_t_j, est_noise);
    }

    Matrix3x3 obse_cov(Matrix3x3 obs_noise, Matrix3x3 est_cov, Matrix3x3 obs_jacob)
    {
        auto t_jacob = transpose_matrix(obs_jacob);

        auto j_ecov = multiply(obs_jacob, est_cov);

        auto j_ecov_t_j = multiply(j_ecov, t_jacob);

        return add(j_ecov_t_j, obs_noise);
    }

    Matrix3x3 kalman_gain(Matrix3x3 est_cov, Matrix3x3 obs_jacob, Matrix3x3 obs_cov)
    {
        auto inv_obs_cov = inverse_matrix(obs_cov);
        auto t_obs_jacob = transpose_matrix(obs_jacob);

        auto es_cv_t_jacob = multiply(est_cov, t_obs_jacob);

        return multiply(inv_obs_cov, es_cv_t_jacob);
    }

    Vector3 update__x(Vector3 est, Vector3 obs, Matrix3x3 kalman_gain)
    {
        auto residual = substract(obs, est);

        auto k_res = multiply(kalman_gain, residual);

        auto x = est.x + k_res.x;
        auto y = est.y + k_res.y;
        auto z = est.z + k_res.z;

        return Vector3(x, y, z);
    }

    Matrix3x3 update__cov(Matrix3x3 kalman_gain, Matrix3x3 est_cov)
    {
        Matrix3x3 i(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        );

        auto i_k = substract(i, kalman_gain);

        return multiply(est_cov, i_k);
    }
}