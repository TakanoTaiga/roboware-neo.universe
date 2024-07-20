#ifndef EKF9_UTILS_HPP_
#define EKF9_UTILS_HPP_

#include "matrix_utils.hpp"
#include <cmath>

namespace motiodom
{
    class Axis9EKF
    {
        public:
        Axis9EKF();
        Vector3 run_ekf9(Vector3 angular_velocity, Vector3 linear_accel, Vector3 magnet);
        
        private:
        Vector3 est;
        Matrix3x3 cov;
        Matrix3x3 est_noise;
        Matrix3x3 obs_noise;
        Matrix3x3 k_gain;
    };

    Vector3 pre_x(Vector3 prev, Vector3 angular_velocity);

    Vector3 obs_model(Vector3 linear_accel, Vector3 mag_);

    Matrix3x3 esti_jacob(Vector3 est, Vector3 angular_velocity);

    Matrix3x3 esti_cov(Matrix3x3 cov, Matrix3x3 est_jacob, Matrix3x3 est_noise);

    Matrix3x3 obse_cov(Matrix3x3 obs_noise, Matrix3x3 est_cov, Matrix3x3 obs_jacob);

    Matrix3x3 kalman_gain(Matrix3x3 est_cov, Matrix3x3 obs_jacob, Matrix3x3 obs_cov);

    Vector3 update__x(Vector3 est, Vector3 obs, Matrix3x3 kalman_gain);

    Matrix3x3 update__cov(Matrix3x3 kalman_gain, Matrix3x3 est_cov);
}

#endif