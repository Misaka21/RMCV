//
// 检测器工厂 - 从配置文件创建检测器
//

#ifndef DETECTOR_FACTORY_HPP
#define DETECTOR_FACTORY_HPP

#include <memory>
#include <string>
#include <vector>

#include "plugin/param/static_config.hpp"
#include "traditional/armor_detector.hpp"
#include "traditional/light_corner_corrector.hpp"
#include "traditional/number_classifier.hpp"

namespace autoaim::detector {

// 从TOML配置文件创建传统装甲板检测器
inline std::unique_ptr<Detector> create_detector_from_config(EnemyColor color) {
    auto config = static_param::parse_file("detector.toml");

    // 二值化阈值
    int binary_thres = static_cast<int>(
        static_param::get_param<int64_t>(config, "Detector.traditional", "binary_thres")
    );
    if (binary_thres == 0)
        binary_thres = 100;

    // PCA角点校正
    bool use_pca = static_param::get_param<bool>(config, "Detector.traditional", "use_pca");

    // 灯条参数
    Detector::LightParams light_params;
    light_params.min_ratio =
        static_param::get_param<double>(config, "Detector.traditional.light", "min_ratio");
    light_params.max_ratio =
        static_param::get_param<double>(config, "Detector.traditional.light", "max_ratio");
    light_params.max_angle =
        static_param::get_param<double>(config, "Detector.traditional.light", "max_angle");
    light_params.color_diff_thresh = static_cast<int>(
        static_param::get_param<int64_t>(config, "Detector.traditional.light", "color_diff_thresh")
    );

    // 默认值
    if (light_params.min_ratio == 0)
        light_params.min_ratio = 0.08;
    if (light_params.max_ratio == 0)
        light_params.max_ratio = 0.4;
    if (light_params.max_angle == 0)
        light_params.max_angle = 40.0;
    if (light_params.color_diff_thresh == 0)
        light_params.color_diff_thresh = 25;

    // 装甲板参数
    Detector::ArmorParams armor_params;
    armor_params.min_light_ratio =
        static_param::get_param<double>(config, "Detector.traditional.armor", "min_light_ratio");
    armor_params.min_small_center_distance = static_param::get_param<double>(
        config,
        "Detector.traditional.armor",
        "min_small_center_distance"
    );
    armor_params.max_small_center_distance = static_param::get_param<double>(
        config,
        "Detector.traditional.armor",
        "max_small_center_distance"
    );
    armor_params.min_large_center_distance = static_param::get_param<double>(
        config,
        "Detector.traditional.armor",
        "min_large_center_distance"
    );
    armor_params.max_large_center_distance = static_param::get_param<double>(
        config,
        "Detector.traditional.armor",
        "max_large_center_distance"
    );
    armor_params.max_angle =
        static_param::get_param<double>(config, "Detector.traditional.armor", "max_angle");

    // 默认值
    if (armor_params.min_light_ratio == 0)
        armor_params.min_light_ratio = 0.6;
    if (armor_params.min_small_center_distance == 0)
        armor_params.min_small_center_distance = 0.8;
    if (armor_params.max_small_center_distance == 0)
        armor_params.max_small_center_distance = 3.2;
    if (armor_params.min_large_center_distance == 0)
        armor_params.min_large_center_distance = 1.8;
    if (armor_params.max_large_center_distance == 0)
        armor_params.max_large_center_distance = 6.4;
    if (armor_params.max_angle == 0)
        armor_params.max_angle = 35.0;

    // 创建检测器
    auto detector = std::make_unique<Detector>(binary_thres, color, light_params, armor_params);

    // 分类器参数
    std::string model_path =
        static_param::get_param<std::string>(config, "Detector.traditional.classify", "model_path");
    std::string label_path =
        static_param::get_param<std::string>(config, "Detector.traditional.classify", "label_path");
    double classify_threshold =
        static_param::get_param<double>(config, "Detector.traditional.classify", "threshold");
    std::string model_type_str =
        static_param::get_param<std::string>(config, "Detector.traditional.classify", "model_type");

    // 默认值
    if (model_path.empty())
        model_path = "lenet.onnx";
    if (label_path.empty())
        label_path = "label.txt";
    if (classify_threshold == 0)
        classify_threshold = 0.8;
    if (model_type_str.empty())
        model_type_str = "lenet";

    // 完整路径
    std::string full_model_path = std::string(ASSET_DIR) + "/" + model_path;
    std::string full_label_path = std::string(ASSET_DIR) + "/" + label_path;
    std::vector<std::string> ignore_classes = { "negative" };
    ClassifierModelType model_type = parse_model_type(model_type_str);

    // 创建分类器 (输入尺寸根据model_type自动确定)
    detector->classifier = std::make_unique<NumberClassifier>(
        full_model_path,
        full_label_path,
        classify_threshold,
        model_type,
        ignore_classes
    );

    // 创建角点校正器
    if (use_pca) {
        detector->corner_corrector = std::make_unique<LightCornerCorrector>();
    }

    return detector;
}

// 手动参数创建检测器（用于测试或自定义配置）
inline std::unique_ptr<Detector> create_detector(
    int binary_thres,
    EnemyColor color,
    const Detector::LightParams& light_params,
    const Detector::ArmorParams& armor_params,
    const std::string& model_path,
    const std::string& label_path,
    double classify_threshold = 0.8,
    ClassifierModelType model_type = ClassifierModelType::LENET,
    const std::vector<std::string>& ignore_classes = { "negative" },
    bool use_pca = true
) {
    auto detector = std::make_unique<Detector>(binary_thres, color, light_params, armor_params);

    detector->classifier = std::make_unique<NumberClassifier>(
        model_path,
        label_path,
        classify_threshold,
        model_type,
        ignore_classes
    );

    if (use_pca) {
        detector->corner_corrector = std::make_unique<LightCornerCorrector>();
    }

    return detector;
}

} // namespace autoaim::detector

#endif // DETECTOR_FACTORY_HPP
