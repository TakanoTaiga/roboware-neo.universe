#ifndef EKF6_UTILS_HPP_
#define EKF6_UTILS_HPP_

#include <chrono>
#include <cmath>
#include <numbers>

#include "matrix_utils.hpp"

namespace motiodom
{
struct AccelAngularEKF
{
  Vector3 est;
  Matrix3x3 cov;
  Matrix3x3 est_noise;
  Matrix2x2 obs_noise;
  Matrix3x2 k_gain;

  AccelAngularEKF(float delta_time = 0.01)
  : est(Vector3(0.0, 0.0, 0.0)),
    cov(Matrix3x3(
      0.0174 * delta_time * delta_time, 0.0, 0.0, 0.0, 0.0174 * delta_time * delta_time, 0.0, 0.0,
      0.0, 0.0174 * delta_time * delta_time)),
    est_noise(Matrix3x3(
      0.0174 * 0.01 * 0.01, 0.0, 0.0, 0.0, 0.0174 * 0.01 * 0.01, 0.0, 0.0, 0.0,
      0.0174 * 0.01 * 0.01)),
    obs_noise(Matrix2x2(0.0, 0.0, 0.0, 0.0)),
    k_gain(Matrix3x2(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
  {
  }
};

Matrix2x3 h() { return Matrix2x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0); }

Matrix3x3 calc_jacob(Vector3 input_matrix, Vector3 estimation_)
{
  auto cos_roll = cos(estimation_.x);
  auto sin_roll = sin(estimation_.x);
  auto cos_pitch = cos(estimation_.y);
  auto sin_pitch = sin(estimation_.y);

  auto m_11 = 1.0 + input_matrix.y * ((cos_roll * sin_pitch) / cos_pitch) -
              input_matrix.z * ((sin_roll * sin_pitch) / cos_pitch);
  auto m_12 = input_matrix.y * (sin_roll / (cos_pitch * cos_pitch)) +
              input_matrix.z * ((cos_roll / (cos_pitch * cos_pitch)));
  auto m_21 = -1.0 * input_matrix.y * sin_roll - input_matrix.z * cos_roll;
  auto m_31 = input_matrix.y * (cos_roll / cos_pitch) - input_matrix.z * (sin_roll / cos_pitch);
  auto m_32 = input_matrix.y * ((sin_roll * sin_pitch) / (cos_pitch * cos_pitch)) +
              input_matrix.z * ((cos_roll * sin_pitch) / (cos_pitch * cos_pitch));

  return Matrix3x3(m_11, m_12, 0.0, m_21, 1.0, 0.0, m_31, m_32, 0.0);
}

Vector3 predict_x(Vector3 input_matrix, Vector3 & estimation_)
{
  auto cos_roll = cos(estimation_.x);
  auto sin_roll = sin(estimation_.x);
  auto cos_pitch = cos(estimation_.y);
  auto sin_pitch = sin(estimation_.y);

  Vector3 estimation(0.0, 0.0, 0.0);
  estimation.x = estimation_.x + input_matrix.x +
                 input_matrix.y * ((sin_roll * sin_pitch) / cos_pitch) +
                 input_matrix.z * ((cos_roll * sin_pitch) / cos_pitch);
  estimation.y = estimation_.y + input_matrix.y * cos_roll - input_matrix.z * sin_roll;
  estimation.z = estimation_.z + input_matrix.z + input_matrix.y * (sin_roll / cos_pitch) +
                 input_matrix.z * (cos_roll / cos_pitch);

  return estimation;
}

Matrix3x3 predict_cov(Matrix3x3 jacob, Matrix3x3 cov_, Matrix3x3 estimation_noise_)
{
  auto t_jacob = transpose_3x3(jacob);
  auto jac_cov = multiply(jacob, cov_);
  Matrix3x3 cov(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  auto multied = multiply(jac_cov, t_jacob);

  cov = add(multied, estimation_noise_);

  return cov;
}

Vector2 update_residual(Vector2 observation, Vector3 estimation_)
{
  Vector2 result(0.0, 0.0);
  Matrix2x3 h_ = h();
  Vector2 h_est = multiply(h_, estimation_);
  result.x = observation.x - h_est.x;
  result.y = observation.y - h_est.y;

  return result;
}

Matrix2x2 update_s(Matrix3x3 cov_, Matrix2x2 observation_noise_)
{
  Matrix2x2 converted = to_2x2(cov_);

  return add(observation_noise_, converted);
}

Matrix3x2 update_kalman_gain(Matrix2x2 s, Matrix3x3 cov_)
{
  Matrix2x3 new_h = h();
  auto t_h = transpose_2x3(new_h);
  auto inv_s = inverse_2x2(s);

  auto cov_t_h = multiply(cov_, t_h);

  return multiply(cov_t_h, inv_s);
}

Vector3 update_x(Vector3 estimation_, Matrix3x2 kalman_gain_, Vector2 residual)
{
  Vector3 kg_res = multiply(kalman_gain_, residual);

  return Vector3(estimation_.x + kg_res.x, estimation_.y + kg_res.y, estimation_.z + kg_res.z);
}

Matrix3x3 update_cov(Matrix3x2 kalman_gain_, Matrix3x3 cov_)
{
  Matrix3x3 i(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

  Matrix2x3 new_h = h();

  Matrix3x3 k_h = multiply(kalman_gain_, new_h);

  Matrix3x3 i_k_h_(
    i.m11 - k_h.m11, i.m12 - k_h.m12, i.m13 - k_h.m13, i.m21 - k_h.m21, i.m22 - k_h.m22,
    i.m23 - k_h.m23, i.m31 - k_h.m31, i.m32 - k_h.m32, i.m33 - k_h.m33);

  return multiply(i_k_h_, cov_);
}

Vector2 obs_model_6(Vector3 linear_accel)
{
  float x_ = 0.0;
  float y_ = 0.0;

  if (linear_accel.z == 0.0) {
    if (linear_accel.y > 0.0) {
      x_ = acos(-1.0) / 2.0;
    } else {
      x_ = -1.0 * acos(-1.0) / 2.0;
    }
  } else {
    x_ = atan(linear_accel.y / linear_accel.z);
  }

  if (sqrt(linear_accel.y * linear_accel.y + linear_accel.z * linear_accel.z) == 0.0) {
    if (-1.0 * linear_accel.x > 0.0) {
      y_ = acos(-1.0) / 2.0;
    } else {
      y_ = -1.0 * acos(-1.0) / 2.0;
    }
  } else {
    y_ = (-1.0 * linear_accel.x) /
         atan(sqrt(linear_accel.y * linear_accel.y + linear_accel.z * linear_accel.z));
  }

  return Vector2(x_, y_);
}

template <typename T>
T to_radian(T degree)
{
  auto pi = acos(-1.0);

  return (degree * pi) / 180.0;
}

}  // namespace motiodom

#endif