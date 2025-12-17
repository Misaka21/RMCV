//
// Detector Factory - Create detector from TOML config
//

#ifndef DETECTOR_FACTORY_HPP
#define DETECTOR_FACTORY_HPP

#include <memory>
#include <string>
#include <vector>

#include "armor_detector.hpp"
#include "light_corner_corrector.hpp"
#include "number_classifier.hpp"
#include "plugin/param/static_config.hpp"

namespace autoaim::detector {

/**
 * @brief Create a traditional armor detector from TOML config
 *
 * Reads parameters from config/detector.toml:
 * - Detector.traditional.binary_thres
 * - Detector.traditional.light.*
 * - Detector.traditional.armor.*
 * - Detector.traditional.classify.*
 *
 * @param color Enemy color to detect
 * @param use_pca Whether to use PCA corner correction
 * @return std::unique_ptr<Detector> Configured detector instance
 */
inline std::unique_ptr<Detector> createDetectorFromConfig(
    EnemyColor color,
    bool use_pca = true
) {
    auto config = static_param::parse_file("detector.toml");

    // Binary threshold
    int binary_thres = static_cast<int>(
        static_param::get_param<int64_t>(config, "Detector.traditional", "binary_thres")
    );
    if (binary_thres == 0) binary_thres = 100;  // Default

    // Light parameters
    Detector::LightParams light_params;
    light_params.min_ratio = static_param::get_param<double>(
        config, "Detector.traditional.light", "min_ratio");
    light_params.max_ratio = static_param::get_param<double>(
        config, "Detector.traditional.light", "max_ratio");
    light_params.max_angle = static_param::get_param<double>(
        config, "Detector.traditional.light", "max_angle");
    light_params.color_diff_thresh = static_cast<int>(
        static_param::get_param<int64_t>(config, "Detector.traditional.light", "color_diff_thresh")
    );

    // Set defaults if not loaded
    if (light_params.min_ratio == 0) light_params.min_ratio = 0.08;
    if (light_params.max_ratio == 0) light_params.max_ratio = 0.4;
    if (light_params.max_angle == 0) light_params.max_angle = 40.0;
    if (light_params.color_diff_thresh == 0) light_params.color_diff_thresh = 25;

    // Armor parameters
    Detector::ArmorParams armor_params;
    armor_params.min_light_ratio = static_param::get_param<double>(
        config, "Detector.traditional.armor", "min_light_ratio");
    armor_params.min_small_center_distance = static_param::get_param<double>(
        config, "Detector.traditional.armor", "min_small_center_distance");
    armor_params.max_small_center_distance = static_param::get_param<double>(
        config, "Detector.traditional.armor", "max_small_center_distance");
    armor_params.min_large_center_distance = static_param::get_param<double>(
        config, "Detector.traditional.armor", "min_large_center_distance");
    armor_params.max_large_center_distance = static_param::get_param<double>(
        config, "Detector.traditional.armor", "max_large_center_distance");
    armor_params.max_angle = static_param::get_param<double>(
        config, "Detector.traditional.armor", "max_angle");

    // Set defaults if not loaded
    if (armor_params.min_light_ratio == 0) armor_params.min_light_ratio = 0.6;
    if (armor_params.min_small_center_distance == 0) armor_params.min_small_center_distance = 0.8;
    if (armor_params.max_small_center_distance == 0) armor_params.max_small_center_distance = 3.2;
    if (armor_params.min_large_center_distance == 0) armor_params.min_large_center_distance = 1.8;
    if (armor_params.max_large_center_distance == 0) armor_params.max_large_center_distance = 6.4;
    if (armor_params.max_angle == 0) armor_params.max_angle = 35.0;

    // Create detector
    auto detector = std::make_unique<Detector>(binary_thres, color, light_params, armor_params);

    // Classifier parameters
    std::string model_path = static_param::get_param<std::string>(
        config, "Detector.traditional.classify", "model_path");
    std::string label_path = static_param::get_param<std::string>(
        config, "Detector.traditional.classify", "label_path");
    double classify_threshold = static_param::get_param<double>(
        config, "Detector.traditional.classify", "threshold");

    // Set defaults
    if (model_path.empty()) model_path = "lenet.onnx";
    if (label_path.empty()) label_path = "label.txt";
    if (classify_threshold == 0) classify_threshold = 0.8;

    // Full paths
    std::string full_model_path = std::string(ASSET_DIR) + "/" + model_path;
    std::string full_label_path = std::string(ASSET_DIR) + "/" + label_path;

    // Ignore classes (hardcoded for now since TOML array parsing is complex)
    std::vector<std::string> ignore_classes = {"negative"};

    // Create classifier
    detector->classifier = std::make_unique<NumberClassifier>(
        full_model_path, full_label_path, classify_threshold, ignore_classes
    );

    // Create corner corrector
    if (use_pca) {
        detector->corner_corrector = std::make_unique<LightCornerCorrector>();
    }

    return detector;
}

/**
 * @brief Create a detector with manual parameters (for testing or custom config)
 */
inline std::unique_ptr<Detector> createDetector(
    int binary_thres,
    EnemyColor color,
    const Detector::LightParams& light_params,
    const Detector::ArmorParams& armor_params,
    const std::string& model_path,
    const std::string& label_path,
    double classify_threshold = 0.8,
    const std::vector<std::string>& ignore_classes = {"negative"},
    bool use_pca = true
) {
    auto detector = std::make_unique<Detector>(binary_thres, color, light_params, armor_params);

    detector->classifier = std::make_unique<NumberClassifier>(
        model_path, label_path, classify_threshold, ignore_classes
    );

    if (use_pca) {
        detector->corner_corrector = std::make_unique<LightCornerCorrector>();
    }

    return detector;
}

}  // namespace autoaim::detector

#endif  // DETECTOR_FACTORY_HPP
