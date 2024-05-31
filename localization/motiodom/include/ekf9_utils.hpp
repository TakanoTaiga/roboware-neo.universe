#ifndef EKF9_UTILS_HPP_
#define EKF9_UTILS_HPP_

#include <cmath>

#include "matrix_utils.hpp"

namespace motiodom
{
struct AAMEKF
{
  Vector3 est;
  Matrix3x3 cov;
  Matrix3x3 est_noise;
  Matrix3x3 obs_noise;
  Matrix3x3 k_gain;

  AAMEKF(float delta_time = 0.01)
  : est(Vector3(0.0, 0.0, 0.0)),
    cov(Matrix3x3(
      0.0174 * delta_time * delta_time, 0.0, 0.0, 0.0, 0.0174 * delta_time * delta_time, 0.0, 0.0,
      0.0, 0.0174 * delta_time * delta_time)),
    est_noise(Matrix3x3(0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01)),
    obs_noise(Matrix3x3(0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1)),
    k_gain(Matrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
  {
  }
};

void predict_x(AAMEKF & ekf, Vector3 angular_velocity)
{
  ekf.est.x =
    ekf.est.x + (angular_velocity.x + angular_velocity.y * tan(ekf.est.y) * sin(ekf.est.x) +
                 angular_velocity.z * tan(ekf.est.y) * cos(ekf.est.x)) *
                  0.01;
  ekf.est.y =
    ekf.est.y + (angular_velocity.y * cos(ekf.est.x) - angular_velocity.z * sin(ekf.est.x)) * 0.01;
  ekf.est.z = ekf.est.z + (angular_velocity.y * (sin(ekf.est.x) / cos(ekf.est.y)) +
                           angular_velocity.z * (cos(ekf.est.x) / cos(ekf.est.y))) *
                            0.01;
}

Vector3 observation_model(Vector3 linear_accel, Vector3 mag_)
{
  Vector3 result(0.0, 0.0, 0.0);

  if (linear_accel.z == 0.0) {
    if (linear_accel.y > 0.0) {
      result.x = acos(-1.0) / 2.0;
    } else {
      result.x = -1.0 * acos(-1.0) / 2.0;
    }
  } else {
    result.x = atan(linear_accel.y / linear_accel.z);
  }

  if (sqrt(linear_accel.y * linear_accel.y + linear_accel.z * linear_accel.z) == 0.0) {
    if (-1.0 * linear_accel.x > 0.0) {
      result.y = acos(-1.0) / 2.0;
    } else {
      result.y = -1.0 * acos(-1.0) / 2.0;
    }
  } else {
    result.y = (-1.0 * linear_accel.x) /
               atan(sqrt(linear_accel.y * linear_accel.y + linear_accel.z * linear_accel.z));
  }

  float above = mag_.x * cos(result.y) + mag_.y * sin(result.y) * sin(result.x) +
                mag_.z * sin(result.y) * cos(result.x);
  float below = mag_.y * cos(result.x) - mag_.z * sin(result.x);

  result.z = atan(above / below);

  return result;
}

Matrix3x3 estimation_jacob(AAMEKF ekf, Vector3 angular_velocity)
{
  return Matrix3x3(
    1.0 + (angular_velocity.x + angular_velocity.y * tan(ekf.est.y) * cos(ekf.est.x) -
           angular_velocity.z * tan(ekf.est.y * sin(ekf.est.x))) *
            0.01,
    (angular_velocity.y * (sin(ekf.est.x) / pow(cos(ekf.est.y), 2)) +
     angular_velocity.z * (cos(ekf.est.x) / pow(cos(ekf.est.y), 2))) *
      0.01,
    0.0, (-1.0 * angular_velocity.y * sin(ekf.est.x) - angular_velocity.z * cos(ekf.est.x)) * 0.01,
    1.0, 0.0,
    (angular_velocity.y * (cos(ekf.est.x) / cos(ekf.est.y)) -
     angular_velocity.z * (sin(ekf.est.x) / cos(ekf.est.y))) *
      0.01,
    (angular_velocity.y * sin(ekf.est.x) * (sin(ekf.est.y) / pow(cos(ekf.est.y), 2)) +
     angular_velocity.z * cos(ekf.est.x) * (sin(ekf.est.y) / pow(cos(ekf.est.y), 2))) *
      0.01,
    1.0);
}

Matrix3x3 estimation_cov(AAMEKF ekf, Matrix3x3 est_jacob)
{
  auto t_jacob = transpose_3x3(est_jacob);

  auto j_cov = multiply(est_jacob, ekf.cov);

  auto j_cov_t_j = multiply(j_cov, t_jacob);

  return add(j_cov_t_j, ekf.est_noise);
}

Matrix3x3 observation_cov(AAMEKF ekf, Matrix3x3 est_cov, Matrix3x3 obs_jacob)
{
  auto t_jacob = transpose_3x3(obs_jacob);

  auto j_ecov = multiply(obs_jacob, est_cov);

  auto j_ecov_t_j = multiply(j_ecov, t_jacob);

  return add(j_ecov_t_j, ekf.obs_noise);
}

void kalman_gain(AAMEKF & ekf, Matrix3x3 est_cov, Matrix3x3 obs_jacob, Matrix3x3 obs_cov)
{
  auto inv_obs_cov = inverse_3x3(obs_cov);
  auto t_obs_jacob = transpose_3x3(obs_jacob);

  auto es_cv_t_jacob = multiply(est_cov, t_obs_jacob);

  ekf.k_gain = multiply(inv_obs_cov, es_cv_t_jacob);
}

void update_x(AAMEKF & ekf, Vector3 obs)
{
  auto residual = substract(obs, ekf.est);

  auto k_res = multiply(ekf.k_gain, residual);

  ekf.est.x = ekf.est.x + k_res.x;
  ekf.est.y = ekf.est.y + k_res.y;
  ekf.est.z = ekf.est.z + k_res.z;
}

void update_cov(AAMEKF & ekf, Matrix3x3 est_cov)
{
  Matrix3x3 i(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

  auto i_k = substract(i, ekf.k_gain);

  ekf.cov = multiply(est_cov, i_k);
}
}  // namespace motiodom

#endif