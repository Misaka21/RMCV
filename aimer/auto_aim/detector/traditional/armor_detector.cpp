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

#include "armor_detector.hpp"
// std
#include <algorithm>
#include <cmath>
#include <execution>
#include <vector>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// 3rd party
#include <fmt/format.h>
// project
#include "types.hpp"

namespace autoaim::detector {
Detector::Detector(const int &bin_thres,
                   const EnemyColor &color,
                   const LightParams &l,
                   const ArmorParams &a)
: binary_thres(bin_thres), detect_color(color), light_params(l), armor_params(a) {}

std::vector<Armor> Detector::detect(const cv::Mat &input) noexcept {
  // 1. 预处理图像
  binary_img = preprocess_image(input);
  // 2. 寻找灯条
  lights_ = find_lights(input, binary_img);
  // 3. 匹配灯条为装甲板
  armors_ = match_lights(lights_);

  if (!armors_.empty() && classifier != nullptr) {
    // 并行处理
    std::for_each(
      std::execution::par, armors_.begin(), armors_.end(), [this, &input](Armor &armor) {
        // 4. 提取数字图像
        armor.number_img = classifier->extract_number(input, armor);
        // 5. 分类
        classifier->classify(input, armor);
        // 6. 校正角点
        if (corner_corrector != nullptr) {
          corner_corrector->correct_corners(armor, gray_img_);
        }
      });

    // 7. 移除忽略的类别
    classifier->erase_ignore_classes(armors_);
  }

  return armors_;
}

cv::Mat Detector::preprocess_image(const cv::Mat &rgb_img) noexcept {
  cv::cvtColor(rgb_img, gray_img_, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img_, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

std::vector<Light> Detector::find_lights(const cv::Mat &rgb_img,
                                        const cv::Mat &binary_img) noexcept {
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  vector<Light> lights;

  for (const auto &contour : contours) {
    if (contour.size() < 6) continue;

    auto light = Light(contour);

    if (is_light(light)) {
      int sum_r = 0, sum_b = 0;
      for (const auto &point : contour) {
        sum_r += rgb_img.at<cv::Vec3b>(point.y, point.x)[0];
        sum_b += rgb_img.at<cv::Vec3b>(point.y, point.x)[2];
      }
      if (std::abs(sum_r - sum_b) / static_cast<int>(contour.size()) >
          light_params.color_diff_thresh) {
        light.color = sum_r > sum_b ? EnemyColor::RED : EnemyColor::BLUE;
      }
      lights.emplace_back(light);
    }
  }
  std::sort(lights.begin(), lights.end(), [](const Light &l1, const Light &l2) {
    return l1.center.x < l2.center.x;
  });
  return lights;
}

bool Detector::is_light(const Light &light) noexcept {
  // 灯条宽高比
  float ratio = light.width / light.length;
  bool ratio_ok = light_params.min_ratio < ratio && ratio < light_params.max_ratio;

  bool angle_ok = light.tilt_angle < light_params.max_angle;

  bool is_light = ratio_ok && angle_ok;

  return is_light;
}

std::vector<Armor> Detector::match_lights(const std::vector<Light> &lights) noexcept {
  std::vector<Armor> armors;
  // 遍历所有灯条配对
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    if (light_1->color != detect_color) continue;
    double max_iter_width = light_1->length * armor_params.max_large_center_distance;

    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_2->color != detect_color) continue;
      if (contain_light(light_1 - lights.begin(), light_2 - lights.begin(), lights)) {
        continue;
      }
      if (light_2->center.x - light_1->center.x > max_iter_width) break;

      auto type = is_armor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// 检查两个灯条之间是否存在其他灯条
bool Detector::contain_light(const int i, const int j, const std::vector<Light> &lights) noexcept {
  const Light &light_1 = lights.at(i), light_2 = lights.at(j);
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);
  double avg_length = (light_1.length + light_2.length) / 2.0;
  double avg_width = (light_1.width + light_2.width) / 2.0;
  // 只检查中间的灯条
  for (int k = i + 1; k < j; k++) {
    const Light &test_light = lights.at(k);

    // 防止数字干扰
    if (test_light.width > 2 * avg_width) {
      continue;
    }
    // 防止红点准星或弹丸干扰
    if (test_light.length < 0.5 * avg_length) {
      continue;
    }

    if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
        bounding_rect.contains(test_light.center)) {
      return true;
    }
  }
  return false;
}

ArmorType Detector::is_armor(const Light &light_1, const Light &light_2) noexcept {
  // 两灯条长度比
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > armor_params.min_light_ratio;

  // 两灯条中心距离（单位：灯条长度）
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (armor_params.min_small_center_distance <= center_distance &&
                             center_distance < armor_params.max_small_center_distance) ||
                            (armor_params.min_large_center_distance <= center_distance &&
                             center_distance < armor_params.max_large_center_distance);

  // 灯条中心连线角度
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < armor_params.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // 判断装甲板类型
  ArmorType type;
  if (is_armor) {
    type = center_distance > armor_params.min_large_center_distance ? ArmorType::LARGE
                                                                    : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  return type;
}

cv::Mat Detector::get_all_numbers_image() const noexcept {
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto &armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

void Detector::draw_results(cv::Mat &img) const noexcept {
  // 绘制灯条

  // for (const auto &light : lights_) {
  //   auto line_color =
  //     light.color == EnemyColor::RED ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 255, 0);
  //   // cv::ellipse(img, light, line_color, 2);
  //   cv::line(img, light.top, light.bottom, line_color, 1);
  // }

  // 绘制装甲板
  for (const auto &armor : armors_) {
    // cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 1);
    // cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1);

    cv::line(
      img, armor.left_light.top, armor.left_light.bottom, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(
      img, armor.right_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(
      img, armor.left_light.top, armor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(img,
             armor.right_light.bottom,
             armor.left_light.bottom,
             cv::Scalar(0, 255, 0),
             1,
             cv::LINE_AA);
  }
  // 显示数字和置信度
  for (const auto &armor : armors_) {
    std::string text =
      fmt::format("{} {}", armor_type_to_string(armor.type), armor.classfication_result);
    cv::putText(
      img, text, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace autoaim::detector
