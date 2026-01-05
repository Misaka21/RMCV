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

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
// std
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <future>
#include <map>
#include <string>
#include <vector>
// 3rd party
#include <fmt/format.h>
// project
#include "number_classifier.hpp"
#include "types.hpp"

namespace autoaim::detector {
NumberClassifier::NumberClassifier(
    const std::string& model_path,
    const std::string& label_path,
    const double thre,
    const ClassifierModelType model_type,
    const std::vector<std::string>& ignore_classes
):
    threshold(thre),
    model_type_(model_type),
    input_size_(get_input_size(model_type)),
    ignore_classes_(ignore_classes) {
    net_ = cv::dnn::readNetFromONNX(model_path);
    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line)) {
        class_names_.push_back(line);
    }
}

cv::Mat NumberClassifier::extract_number(const cv::Mat& src, const Armor& armor) const noexcept {
    // 图像中灯条长度
    static const int light_length = 12;
    // 透视变换后的图像尺寸
    static const int warp_height = 28;
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;
    // 数字ROI尺寸
    static const cv::Size roi_size(20, 28);

    // 透视变换
    cv::Point2f lights_vertices[4] = { armor.left_light.bottom,
                                       armor.left_light.top,
                                       armor.right_light.top,
                                       armor.right_light.bottom };

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // 获取ROI
    number_image =
        number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // 二值化
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 根据配置的输入尺寸调整大小 (如果不等于ROI尺寸才resize)
    if (input_size_ != roi_size) {
        cv::resize(number_image, number_image, input_size_);
    }

    return number_image;
}

std::pair<double, int> NumberClassifier::decode_output(const cv::Mat& output_row) const noexcept {
    double confidence;
    cv::Point class_id_point;

    if (model_type_ == ClassifierModelType::MLP) {
        // MLP模型输出logits，需要softmax
        float max_val = *std::max_element(output_row.begin<float>(), output_row.end<float>());
        cv::Mat softmax_prob;
        cv::exp(output_row - max_val, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    } else {
        // LeNet模型输出已是概率，直接取最大值
        minMaxLoc(output_row.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    }

    return { confidence, class_id_point.x };
}

void NumberClassifier::classify(const cv::Mat& src, Armor& armor) noexcept {
    // 归一化 (使用convertTo更高效)
    cv::Mat input;
    armor.number_img.convertTo(input, CV_32F, 1.0 / 255.0);

    // 创建blob
    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob);

    // 设置神经网络输入
    mutex_.lock();
    net_.setInput(blob);

    // 前向传播
    cv::Mat outputs = net_.forward().clone();
    mutex_.unlock();

    // 解码输出
    auto [confidence, label_id] = decode_output(outputs);

    armor.confidence = confidence;
    armor.number = class_names_[label_id];
    armor.classfication_result = fmt::format("{}:{:.1f}%", armor.number, armor.confidence * 100.0);
}

void NumberClassifier::classify_batch(std::vector<Armor>& armors) noexcept {
    if (armors.empty())
        return;

    // 收集所有数字图像并归一化
    std::vector<cv::Mat> inputs;
    inputs.reserve(armors.size());
    for (auto& armor: armors) {
        cv::Mat input;
        armor.number_img.convertTo(input, CV_32F, 1.0 / 255.0);
        inputs.push_back(input);
    }

    // 批量创建blob (将多张图像合并为一个batch)
    cv::Mat blob;
    cv::dnn::blobFromImages(inputs, blob);

    // 一次性前向推理
    net_.setInput(blob);
    cv::Mat outputs = net_.forward();

    // 解码每个装甲板的输出
    for (size_t i = 0; i < armors.size(); ++i) {
        cv::Mat output_row = outputs.row(static_cast<int>(i));
        auto [confidence, label_id] = decode_output(output_row);

        armors[i].confidence = confidence;
        armors[i].number = class_names_[label_id];
        armors[i].classfication_result =
            fmt::format("{}:{:.1f}%", armors[i].number, armors[i].confidence * 100.0);
    }
}

void NumberClassifier::erase_ignore_classes(std::vector<Armor>& armors) noexcept {
    armors.erase(
        std::remove_if(
            armors.begin(),
            armors.end(),
            [this](const Armor& armor) {
                if (armor.confidence < threshold) {
                    return true;
                }

                for (const auto& ignore_class: ignore_classes_) {
                    if (armor.number == ignore_class) {
                        return true;
                    }
                }

                bool mismatch_armor_type = false;
                if (armor.type == ArmorType::LARGE) {
                    mismatch_armor_type = armor.number == "outpost" || armor.number == "2"
                        || armor.number == "sentry";
                } else if (armor.type == ArmorType::SMALL) {
                    mismatch_armor_type = armor.number == "1" || armor.number == "base";
                }
                return mismatch_armor_type;
            }
        ),
        armors.end()
    );
}

} // namespace autoaim::detector
