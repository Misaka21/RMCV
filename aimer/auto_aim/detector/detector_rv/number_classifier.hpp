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

#ifndef ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_

// std
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>
// third party
#include <opencv2/opencv.hpp>
// project
#include "types.hpp"

namespace autoaim::detector {

// 数字分类模型类型
enum class ClassifierModelType {
    LENET, // 输出已是概率，直接取最大值
    MLP // 输出是logits，需要softmax
};

// 从字符串解析模型类型
inline ClassifierModelType parse_model_type(const std::string& type_str) {
    if (type_str == "mlp" || type_str == "MLP") {
        return ClassifierModelType::MLP;
    }
    return ClassifierModelType::LENET; // 默认lenet
}

// 根据模型类型获取输入尺寸
inline cv::Size get_input_size(ClassifierModelType model_type) {
    switch (model_type) {
        case ClassifierModelType::MLP:
            return cv::Size(20, 28); // MLP使用原始ROI尺寸
        case ClassifierModelType::LENET:
        default:
            return cv::Size(28, 28); // LeNet需要resize到28x28
    }
}

// 装甲板数字分类器，基于MLP模型
class NumberClassifier {
public:
    NumberClassifier(
        const std::string& model_path,
        const std::string& label_path,
        const double threshold,
        const ClassifierModelType model_type = ClassifierModelType::LENET,
        const std::vector<std::string>& ignore_classes = {}
    );

    // 从图像中提取数字区域
    cv::Mat extract_number(const cv::Mat& src, const Armor& armor) const noexcept;

    // 分类装甲板数字 (单个)
    void classify(const cv::Mat& src, Armor& armor) noexcept;

    // 批量分类装甲板数字 (推荐使用，性能更好)
    void classify_batch(std::vector<Armor>& armors) noexcept;

    // 移除忽略的类别
    void erase_ignore_classes(std::vector<Armor>& armors) noexcept;

    double threshold;

private:
    std::mutex mutex_;
    cv::dnn::Net net_;
    ClassifierModelType model_type_;
    cv::Size input_size_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;

    // 对单行输出应用softmax并返回置信度和类别ID
    std::pair<double, int> decode_output(const cv::Mat& output_row) const noexcept;
};
} // namespace autoaim::detector
#endif // ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_
