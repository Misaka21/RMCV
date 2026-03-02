// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
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

#ifndef ARMOR_DETECTOR_DETECTOR_HPP_
#define ARMOR_DETECTOR_DETECTOR_HPP_

// std
#include <cmath>
#include <string>
#include <vector>
// third party
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
// project
#include "../common/detector_interface.hpp"
#include "light_corner_corrector.hpp"
#include "number_classifier.hpp"
#include "types.hpp"

namespace autoaim::detector {

/**
 * @brief 传统装甲板检测器
 *
 * 基于灯条匹配的装甲板检测算法
 * 实现 DetectorInterface 接口，可通过工厂创建
 */
class Detector : public DetectorInterface {
public:
    struct LightParams {
        double min_ratio = 0.08;
        double max_ratio = 0.4;
        double max_angle = 40.0;
        int color_diff_thresh = 25;
    };

    struct ArmorParams {
        double min_light_ratio = 0.6;
        double min_small_center_distance = 0.8;
        double max_small_center_distance = 3.2;
        double min_large_center_distance = 1.8;
        double max_large_center_distance = 6.4;
        double max_angle = 35.0;
    };

    Detector(int bin_thres, EnemyColor color, const LightParams& l, const ArmorParams& a);

    /**
     * @brief 从配置文件创建检测器
     * @param color 敌方颜色
     * @param config_file 配置文件名 (默认 armor_detector.toml)
     */
    static std::unique_ptr<Detector> from_config(
        EnemyColor color,
        const std::string& config_file = "armor_detector.toml"
    );

    // ============================================================================
    // IDetector 接口实现
    // ============================================================================

    std::vector<DetectedArmor> detect(const cv::Mat& input) override;
    void set_enemy_color(EnemyColor color) override { detect_color = color; }
    EnemyColor get_enemy_color() const override { return detect_color; }
    cv::Mat debug_image() const override;

    // ============================================================================
    // 内部方法 (供调试和扩展使用)
    // ============================================================================

    cv::Mat preprocess_image(const cv::Mat& input) noexcept;
    std::vector<Light> find_lights(const cv::Mat& rbg_img, const cv::Mat& binary_img) noexcept;
    std::vector<Armor> match_lights(const std::vector<Light>& lights) noexcept;

    // 调试用
    cv::Mat get_all_numbers_image() const noexcept;
    void draw_results(cv::Mat& img) const noexcept;

    // 参数 (public for factory configuration)
    int binary_thres;
    EnemyColor detect_color;
    LightParams light_params;
    ArmorParams armor_params;

    std::unique_ptr<NumberClassifier> classifier;
    std::unique_ptr<LightCornerCorrector> corner_corrector;

    // 调试信息
    cv::Mat binary_img;

private:
    bool is_light(const Light& possible_light) noexcept;
    bool contain_light(int i, int j, const std::vector<Light>& lights) noexcept;
    ArmorType is_armor(const Light& light_1, const Light& light_2) noexcept;

    // 内部检测 (返回内部Armor类型)
    std::vector<Armor> detect_internal(const cv::Mat& input) noexcept;

    cv::Mat gray_img_;
    std::vector<Light> lights_;
    std::vector<Armor> armors_;
};

} // namespace autoaim::detector

#endif // ARMOR_DETECTOR_DETECTOR_HPP_
