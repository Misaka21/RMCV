// Maintained by Shenglin Qin, Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
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

#ifndef ARMOR_DETECTOR_LIGHT_CORNER_CORRECTOR_HPP_
#define ARMOR_DETECTOR_LIGHT_CORNER_CORRECTOR_HPP_

// opencv
#include <opencv2/opencv.hpp>
// project
#include "types.hpp"

namespace autoaim::detector {

struct SymmetryAxis {
    cv::Point2f centroid;
    cv::Point2f direction;
    float mean_val; // 平均亮度
};

// 灯条角点校正器
// 使用PCA算法找到灯条的对称轴，然后沿对称轴根据亮度梯度找到角点
class LightCornerCorrector {
public:
    explicit LightCornerCorrector() noexcept {}

    // 校正装甲板灯条的角点
    void correct_corners(Armor& armor, const cv::Mat& gray_img);

private:
    // 查找灯条的对称轴
    SymmetryAxis find_symmetry_axis(const cv::Mat& gray_img, const Light& light);

    // 查找灯条的角点
    cv::Point2f find_corner(
        const cv::Mat& gray_img,
        const Light& light,
        const SymmetryAxis& axis,
        std::string order
    );
};

} // namespace autoaim::detector
#endif // ARMOR_DETECTOR_LIGHT_CORNER_CORRECTOR_HPP_
