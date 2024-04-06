// Copyright 2024 Taiga Takano
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

#include "tailoc_ros2/ndt_cpp.hpp"

namespace ndt_cpp
{

mat3x3 makeTransformationMatrix(const double& tx, const double& ty, const double& rad){
    mat3x3 mat = {
        cos(rad), sin(rad) * -1.0, tx,
        sin(rad), cos(rad)       , ty,
        0.0, 0.0, 1.0
    };
    return mat;
}

mat3x3 multiplyMatrices3x3x2(const mat3x3& mat1, const mat3x3& mat2) {
    mat3x3 result;
    result.a = mat1.a * mat2.a + mat1.b * mat2.d + mat1.c * mat2.g;
    result.b = mat1.a * mat2.b + mat1.b * mat2.e + mat1.c * mat2.h;
    result.c = mat1.a * mat2.c + mat1.b * mat2.f + mat1.c * mat2.i;

    result.d = mat1.d * mat2.a + mat1.e * mat2.d + mat1.f * mat2.g;
    result.e = mat1.d * mat2.b + mat1.e * mat2.e + mat1.f * mat2.h;
    result.f = mat1.d * mat2.c + mat1.e * mat2.f + mat1.f * mat2.i;

    result.g = mat1.g * mat2.a + mat1.h * mat2.d + mat1.i * mat2.g;
    result.h = mat1.g * mat2.b + mat1.h * mat2.e + mat1.i * mat2.h;
    result.i = mat1.g * mat2.c + mat1.h * mat2.f + mat1.i * mat2.i;

    return result;
}

point3 multiplyMatrixPoint3(const mat3x3& mat, const point3& vec) {
    point3 result;
    result.x = mat.a * vec.x + mat.b * vec.y + mat.c * vec.z;
    result.y = mat.d * vec.x + mat.e * vec.y + mat.f * vec.z;
    result.z = mat.g * vec.x + mat.h * vec.y + mat.i * vec.z;
    return result;
}

double multtiplyPowPoint3(const point3& vec){
    return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}

void addMat3x3(mat3x3& mat1, const mat3x3& mat2){
    mat1.a += mat2.a;
    mat1.b += mat2.b;
    mat1.c += mat2.c;

    mat1.d += mat2.d;
    mat1.e += mat2.e;
    mat1.f += mat2.f;

    mat1.g += mat2.g;
    mat1.h += mat2.h;
    mat1.i += mat2.i;
}

void addPoint3(point3& point1, const point3& point2){
    point1.x += point2.x;
    point1.y += point2.y;
    point1.z += point2.z;
}

point3 solve3x3(const mat3x3& m, const point3& p) {
    double A[3][4] = {
        {m.a, m.b, m.c, p.x},
        {m.d, m.e, m.f, p.y},
        {m.g, m.h, m.i, p.z}
    };

    int n = 3;

    for (int i = 0; i < n; i++) {
        double maxEl = std::abs(A[i][i]);
        int maxRow = i;
        for (int k = i+1; k < n; k++) {
            if (std::abs(A[k][i]) > maxEl) {
                maxEl = std::abs(A[k][i]);
                maxRow = k;
            }
        }

        for (int k = i; k < n+1;k++) {
            double tmp = A[maxRow][k];
            A[maxRow][k] = A[i][k];
            A[i][k] = tmp;
        }

        for (int k = i+1; k < n; k++) {
            double c = -A[k][i] / A[i][i];
            for (int j = i; j < n+1; j++) {
                if (i == j) {
                    A[k][j] = 0.0f;
                } else {
                    A[k][j] += c * A[i][j];
                }
            }
        }
    }

    point3 solution;
    solution.z = A[2][3] / A[2][2];
    solution.y = (A[1][3] - A[1][2] * solution.z) / A[1][1];
    solution.x = (A[0][3] - A[0][2] * solution.z - A[0][1] * solution.y) / A[0][0];

    return solution;
}

mat3x3 expmap(const point3& point){
    auto t = point.z;
    auto c = cos(t);
    auto s = sin(t);

    mat2x2 R {
        c, s * -1.0f,
        s, c
    };

    mat3x3 T {
        R.a, R.b, point.x,
        R.c, R.d, point.y,
        0.0f, 0.0f, 1.0f
    };

    return T;
}

point2 transformPointCopy(const mat3x3& mat, const point2& point) {
    point2 transformedPoint;

    transformedPoint.x = mat.a * point.x + mat.b * point.y + mat.c;
    transformedPoint.y = mat.d * point.x + mat.e * point.y + mat.f;

    return transformedPoint;
}

mat3x3 inverse3x3Copy(const mat3x3& mat){
    const auto a = 1.0 / (
        mat.a * mat.e * mat.i + 
        mat.b * mat.f * mat.g +
        mat.c * mat.d * mat.h -
        mat.c * mat.e * mat.g -
        mat.b * mat.d * mat.i -
        mat.a * mat.f * mat.h
        );

    mat3x3 inv_mat;
    inv_mat.a = mat.e * mat.i - mat.f * mat.h;
    inv_mat.b = mat.b * mat.i - mat.c * mat.h;
    inv_mat.c = mat.b * mat.f - mat.c * mat.e;

    inv_mat.d = mat.d * mat.i - mat.f * mat.g;
    inv_mat.e = mat.a * mat.i - mat.c * mat.g;
    inv_mat.f = mat.a * mat.f - mat.c * mat.d;

    inv_mat.g = mat.d * mat.h - mat.e * mat.g;
    inv_mat.h = mat.a * mat.h - mat.b * mat.g;
    inv_mat.i = mat.a * mat.e - mat.b * mat.d;


    inv_mat.a = inv_mat.a * a;
    inv_mat.b = inv_mat.b * a * -1.0f;
    inv_mat.c = inv_mat.c * a;

    inv_mat.d = inv_mat.d * a * -1.0f;
    inv_mat.e = inv_mat.e * a;
    inv_mat.f = inv_mat.f * a * -1.0f;

    inv_mat.g = inv_mat.g * a;
    inv_mat.h = inv_mat.h * a * -1.0f;
    inv_mat.i = inv_mat.i * a;

    return inv_mat;
}

point2 skewd(const point2& input_point){
    const point2 skewd_point {
        input_point.y,
        input_point.x * -1.0f
    };
    return skewd_point;
}

mat3x3 transpose(const mat3x3& input_mat){
    const mat3x3 transpose_mat{
        input_mat.a, input_mat.d, input_mat.g,
        input_mat.b, input_mat.e, input_mat.h,
        input_mat.c, input_mat.f, input_mat.i
    };
    return transpose_mat;
}

point2 compute_mean(const std::vector<point2>& points){
    point2 mean;
    mean.x = 0.0f;
    mean.y = 0.0f;
    for(const auto& point : points){
        mean.x += point.x;
        mean.y += point.y;
    }
    mean.x = mean.x / (double)points.size();
    mean.y = mean.y / (double)points.size();
    return mean;
}

mat2x2 compute_covariance(const std::vector<point2>& points, const point2& mean){
    auto point_size = points.size();
    auto vxx = 0.0f;
    auto vxy = 0.0f;
    auto vyy = 0.0f;

    for(const auto& point : points){
        const auto dx = point.x - mean.x;
        const auto dy = point.y - mean.y;
        vxx += dx * dx;
        vxy += dx * dy;
        vyy += dy * dy;
    }

    mat2x2 cov;
    cov.a = vxx / point_size;
    cov.b = vxy / point_size;
    cov.c = vxy / point_size;
    cov.d = vyy / point_size;
    return cov;
}

std::vector<mat2x2> compute_ndt_points(const ndtParam& param, std::vector<point2>& points){
    // auto N = 10;
    
    const auto point_size = points.size();

    std::vector<std::pair<double, size_t>> distances(point_size);
    std::vector<point2> compute_points(param.ndt_sample_num_point);

    std::vector<mat2x2> covs;

    for(auto &point : points){
        for(size_t i = 0; i < point_size; i++){
            auto dx = points[i].x - point.x;
            auto dy = points[i].y - point.y;
            distances[i] = { dx * dx + dy * dy, i };
        }
        std::nth_element(distances.begin(), distances.begin() + param.ndt_sample_num_point, distances.end());
        for (size_t i = 0; i < param.ndt_sample_num_point; i++){
            compute_points[i] = points[distances[i].second];
        }

        const auto mean = compute_mean(compute_points);
        const auto cov = compute_covariance(compute_points, mean);

        point = mean;
        covs.push_back(cov);
    }

    return covs;
}

void ndt_scan_matching(const ndtParam& param, const rclcpp::Logger& logger, mat3x3& trans_mat, const std::vector<point2>& source_points, const std::vector<point2>& target_points, const std::vector<mat2x2>& target_covs){
    const double max_distance2 = 1.0f * 1.0f;
    
    const size_t target_points_size = target_points.size();
    const size_t source_points_size = source_points.size();
    std::vector<std::pair<double, size_t>> distances(target_points_size);

    for(size_t iter = 0; iter < param.ndt_max_iteration; iter++){
        mat3x3 H_Mat {
            0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f
        };

        point3 b_Point {
            0.0f, 0.0f, 0.0f
        };

        for(auto point_iter = 0; point_iter < source_points_size; point_iter += param.ndt_matching_step){
            point2 query_point = transformPointCopy(trans_mat, source_points[point_iter]);
            for(auto i = 0; i < target_points_size; i++){
                auto dx = target_points[i].x - query_point.x;
                auto dy = target_points[i].y - query_point.y;
                distances[i] = { dx * dx + dy * dy, i };
            }

            auto target_iter = std::min_element(distances.begin(), distances.end());

            const point2 target_point = target_points[target_iter->second];
            const double target_distance = target_iter->first;
            const mat2x2 target_cov = target_covs[target_iter->second];

            if(target_distance > max_distance2){continue;}

            const auto identity_plus_cov = mat3x3{
                target_cov.a, target_cov.b, 0.0f,
                target_cov.c, target_cov.d, 0.0f,
                0.0f, 0.0f, 1.0f
            };

            const mat3x3 target_cov_inv = inverse3x3Copy(identity_plus_cov); //IM


            const auto error = point3{
                target_point.x - query_point.x,
                target_point.y - query_point.y,
                0.0f
            };

            const point2 v_point = transformPointCopy(trans_mat, skewd(source_points[point_iter]));

            const auto mat_J = mat3x3{
                trans_mat.a * -1.0f, trans_mat.b * -1.0f, v_point.x,
                trans_mat.d * -1.0f, trans_mat.e * -1.0f, v_point.y,
                trans_mat.g * -1.0f, trans_mat.h * -1.0f, trans_mat.i * -1.0f
            };

            const mat3x3 mat_J_T = transpose(mat_J);

            addMat3x3(H_Mat, multiplyMatrices3x3x2(mat_J_T, multiplyMatrices3x3x2(target_cov_inv, mat_J)));

            addPoint3(b_Point, multiplyMatrixPoint3(mat_J_T, multiplyMatrixPoint3(target_cov_inv, error)));

        } 
        b_Point.x *= -1.0;
        b_Point.y *= -1.0;
        b_Point.z *= -1.0;
        const point3 delta = solve3x3(H_Mat,b_Point);
        trans_mat = multiplyMatrices3x3x2(trans_mat, expmap(delta));

        if(multtiplyPowPoint3(delta) < param.ndt_precision){
            if(param.enable_debug){
                RCLCPP_INFO_STREAM(logger, 
                    "END NDT. ITER: " << iter << 
                    " dx: " << trans_mat.c << 
                    " dy: " << trans_mat.f << 
                    " dz: " << atan(trans_mat.d / trans_mat.a) * 57.295779 << "°"
                );
            }
            
            return;
        }
    }
    if(param.enable_debug){
        RCLCPP_WARN_STREAM(logger,"INACCURATE ODOMETRY");
    }
}
}