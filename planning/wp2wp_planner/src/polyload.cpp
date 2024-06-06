// Copyright 2024 Hakoroboken
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

#include "wp2wp_planner/polyload.hpp"

namespace wp2wp_planner
{
namespace polyload
{
    boost_type::polygon_2d_lf polyload(std::string path)
    {
        std::ifstream infile(path);
        if (!infile) {
            std::cerr << "Failed to open file." << std::endl;
            exit(1);
        }
           
        std::string line;
        std::vector<boost_type::point_2d_lf> points;
        while (std::getline(infile, line)) {
            std::istringstream ss(line);
            double x, y;
            char comma;
            if (ss >> x >> comma >> y) {
                points.emplace_back(x, y);
            } else {
                std::cerr << "Invalid line format: " << line << std::endl;
                exit(1);
            }
        }
        boost_type::polygon_2d_lf ply2d;
        boost::geometry::assign_points(ply2d, points);
        return ply2d;
    }
}
}
