// Created by Chengfu Zou on 2023.10.26
// Maintained by Chengfu Zou
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

#ifndef ARMOR_DETECTOR_TYPES_HPP_
#define ARMOR_DETECTOR_TYPES_HPP_

// std
#include <algorithm>
#include <cassert>
#include <numeric>
#include <string>
// 3rd party
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// 公共类型
#include "aimer/auto_aim/common/types.hpp"

namespace autoaim::detector {

// 从 autoaim 命名空间导入类型
using autoaim::EnemyColor;
using autoaim::ArmorType;

// 15度对应的弧度 (内部使用)
constexpr double FIFTEEN_DEGREE_RAD = 15 * CV_PI / 180;

// ============================================================================
// 内部类型 - 仅供传统检测器使用
// ============================================================================

/**
 * @brief 灯条结构体 - 传统检测器内部使用
 */
struct Light : public cv::RotatedRect {
    Light() = default;
    explicit Light(const std::vector<cv::Point>& contour)
        : cv::RotatedRect(cv::minAreaRect(contour)), color(EnemyColor::GRAY) {
        assert(!contour.empty());

        center = std::accumulate(
            contour.begin(), contour.end(), cv::Point2f(0, 0),
            [n = static_cast<float>(contour.size())](const cv::Point2f& a, const cv::Point& b) {
                return a + cv::Point2f(b.x, b.y) / n;
            });

        cv::Point2f p[4];
        this->points(p);
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);

        axis = top - bottom;
        axis = axis / cv::norm(axis);

        // 计算倾斜角度（灯条与水平线的夹角）
        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
    }

    EnemyColor color;
    cv::Point2f top, bottom, center;
    cv::Point2f axis;
    double length;
    double width;
    float tilt_angle;
};

/**
 * @brief 装甲板结构体 - 传统检测器内部使用
 *
 * 包含灯条对信息，用于检测过程
 * 最终输出时转换为 DetectedArmor
 */
struct Armor {
    static constexpr int N_LANDMARKS = 4;
    static constexpr int N_LANDMARKS_2 = N_LANDMARKS * 2;

    Armor() = default;
    Armor(const Light& l1, const Light& l2) {
        if (l1.center.x < l2.center.x) {
            left_light = l1;
            right_light = l2;
        } else {
            left_light = l2;
            right_light = l1;
        }
        center = (left_light.center + right_light.center) / 2;
    }

    // 构建物体坐标系中的点，从左上角开始逆时针排列
    template<typename PointType>
    static std::vector<PointType> buildObjectPoints(double w, double h) noexcept {
        if constexpr (N_LANDMARKS == 4) {
            return {
                PointType(0, w / 2, h / 2),    // 左上
                PointType(0, w / 2, -h / 2),   // 左下
                PointType(0, -w / 2, -h / 2),  // 右下
                PointType(0, -w / 2, h / 2)    // 右上
            };
        } else {
            return {
                PointType(0, w / 2, h / 2),
                PointType(0, w / 2, 0),
                PointType(0, w / 2, -h / 2),
                PointType(0, -w / 2, -h / 2),
                PointType(0, -w / 2, 0),
                PointType(0, -w / 2, h / 2)
            };
        }
    }

    // 获取关键点，从左上角开始逆时针排列
    std::vector<cv::Point2f> landmarks() const {
        if constexpr (N_LANDMARKS == 4) {
            return {left_light.top, left_light.bottom, right_light.bottom, right_light.top};
        } else {
            return {
                left_light.top, left_light.center, left_light.bottom,
                right_light.bottom, right_light.center, right_light.top
            };
        }
    }

    // 灯条对
    Light left_light, right_light;
    cv::Point2f center;
    ArmorType type = ArmorType::INVALID;

    // 数字识别
    cv::Mat number_img;
    std::string number;
    float confidence = 0.5f;
    std::string classfication_result;
};

} // namespace autoaim::detector

#endif // ARMOR_DETECTOR_TYPES_HPP_
