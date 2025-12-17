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
#include "light_corner_corrector.hpp"
#include "number_classifier.hpp"
#include "types.hpp"

namespace autoaim::detector {

// 传统装甲板检测器
class Detector {
public:
  struct LightParams {
    // 宽高比
    double min_ratio;
    double max_ratio;
    // 垂直角度
    double max_angle;
    // 颜色判断
    int color_diff_thresh;
  };

  struct ArmorParams {
    double min_light_ratio;
    // 灯条对距离
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // 水平角度
    double max_angle;
  };

  Detector(const int &bin_thres, const EnemyColor &color, const LightParams &l,
           const ArmorParams &a);

  std::vector<Armor> detect(const cv::Mat &input) noexcept;

  cv::Mat preprocess_image(const cv::Mat &input) noexcept;
  std::vector<Light> find_lights(const cv::Mat &rbg_img,
                                const cv::Mat &binary_img) noexcept;
  std::vector<Armor> match_lights(const std::vector<Light> &lights) noexcept;

  // 调试用
  cv::Mat get_all_numbers_image() const noexcept;
  void draw_results(cv::Mat &img) const noexcept;

  // 参数
  int binary_thres;
  EnemyColor detect_color;
  LightParams light_params;
  ArmorParams armor_params;

  std::unique_ptr<NumberClassifier> classifier;
  std::unique_ptr<LightCornerCorrector> corner_corrector;

  // 调试信息
  cv::Mat binary_img;

private:
  bool is_light(const Light &possible_light) noexcept;
  bool contain_light(const int i,const int j,const std::vector<Light> &lights) noexcept;
  ArmorType is_armor(const Light &light_1, const Light &light_2) noexcept;

  cv::Mat gray_img_;

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

} // namespace autoaim::detector

#endif // ARMOR_DETECTOR_DETECTOR_HPP_
