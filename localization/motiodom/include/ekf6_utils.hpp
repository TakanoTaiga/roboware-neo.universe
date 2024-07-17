#ifndef EKF6_UTILS_HPP_
#define EKF6_UTILS_HPP_

#include "matrix_utils.hpp"

#include <numbers>
#include <cmath>
#include <chrono>

namespace motiodom
{
    class Axis6EKF
    {
        public:
        Axis6EKF();
        Vector3 run_ekf6(Vector3 input_matrix, Vector3 linear_accel);

        private:
        Vector3 est;
        Matrix3x3 cov;
        Matrix3x3 est_noise;
        Matrix2x2 obs_noise;
        Matrix3x2 k_gain;
    };

    Matrix2x3 h();
    
    Matrix3x3 calc_jacob(const Vector3 input_matrix, const Vector3 estimation_);

    Vector3 predict_x(const Vector3 input_matrix, const Vector3 estimation_);

    Matrix3x3 predict_cov(const Matrix3x3 jacob, const Matrix3x3 cov_, const Matrix3x3 estimation_noise_);

    Vector2 update_residual(const Vector2 observation, const Vector3 estimation_);

    Matrix2x2 update_s(const Matrix3x3 cov_, const Matrix2x2 observation_noise_);
    
    Matrix3x2 update_kalman_gain(const Matrix2x2 s, const Matrix3x3 cov_);

    Vector3 update_x(const Vector3 estimation_, const Matrix3x2 kalman_gain_, const Vector2 residual);

    Matrix3x3 update_cov(const Matrix3x2 kalman_gain_, const Matrix3x3 cov_);

    Vector2 obs_model_6(const Vector3 linear_accel);
}

#endif