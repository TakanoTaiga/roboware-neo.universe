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

#ifndef NDT_CPP_NODE_HPP_
#define NDT_CPP_NODE_HPP_

#include <vector>

namespace ndt_cpp
{

struct point2{
    double x, y;
};

struct point3{
    double x, y, z;
};

struct mat2x2{
    double a, b;
    double c, d;
};

struct mat3x3{
    double a, b, c;
    double d, e, f;
    double g, h, i;
};

std::vector<mat2x2> compute_ndt_points(std::vector<point2>& points);
void ndt_scan_matching(mat3x3& trans_mat, const std::vector<point2>& source_points, const std::vector<point2>& target_points, const std::vector<mat2x2>& target_covs);

}


#endif