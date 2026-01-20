//
// 能量机关检测器工厂实现
//

#include "detector_factory.hpp"

#include <stdexcept>

#include "plugin/param/static_config.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/debug/log.hpp"

#include "traditional/traditional_detector.hpp"

namespace autobuff::detector {

std::unique_ptr<BuffDetectorInterface> create_detector_from_config(
    EnemyColor color,
    const std::string& config_file) {

    // 读取检测器类型
    auto config = static_param::parse_file(config_file);
    auto type_str = static_param::get_param<std::string>(config, "Detector", "type")
                        .value_or("traditional");

    debug::print("info", "BuffDetector", "Creating detector: type={}", type_str);

    if (type_str == "traditional") {
        return create_traditional_detector(color, config_file);
    } else if (type_str == "yolo") {
        // TODO: 实现 YOLO 检测器
        throw std::runtime_error("YOLO detector not implemented yet");
    } else {
        throw std::runtime_error("Unknown detector type: " + type_str);
    }
}

std::unique_ptr<BuffDetectorInterface> create_traditional_detector(
    EnemyColor color,
    const std::string& config_file) {

    auto config = static_param::parse_file(config_file);

    TraditionalDetectorConfig detector_config;

    // 读取配置参数
    detector_config.gray_threshold_red =
        static_param::get_param<int>(config, "TraditionalDetector", "gray_threshold_red")
            .value_or(80);
    detector_config.gray_threshold_blue =
        static_param::get_param<int>(config, "TraditionalDetector", "gray_threshold_blue")
            .value_or(80);

    // R标参数
    detector_config.r_area_min =
        static_param::get_param<double>(config, "TraditionalDetector", "r_area_min")
            .value_or(100);
    detector_config.r_area_max =
        static_param::get_param<double>(config, "TraditionalDetector", "r_area_max")
            .value_or(5000);
    detector_config.r_circularity_min =
        static_param::get_param<double>(config, "TraditionalDetector", "r_circularity_min")
            .value_or(0.2);
    detector_config.r_circularity_max =
        static_param::get_param<double>(config, "TraditionalDetector", "r_circularity_max")
            .value_or(0.9);

    // 靶心参数
    detector_config.target_area_min =
        static_param::get_param<double>(config, "TraditionalDetector", "target_area_min")
            .value_or(60);
    detector_config.target_area_max =
        static_param::get_param<double>(config, "TraditionalDetector", "target_area_max")
            .value_or(6000);
    detector_config.target_aspect_min =
        static_param::get_param<double>(config, "TraditionalDetector", "target_aspect_min")
            .value_or(0.99);
    detector_config.target_aspect_max =
        static_param::get_param<double>(config, "TraditionalDetector", "target_aspect_max")
            .value_or(1.55);

    // 箭头参数
    detector_config.arrow_area_min =
        static_param::get_param<double>(config, "TraditionalDetector", "arrow_area_min")
            .value_or(100);
    detector_config.arrow_area_max =
        static_param::get_param<double>(config, "TraditionalDetector", "arrow_area_max")
            .value_or(4000);
    detector_config.arrow_aspect_min =
        static_param::get_param<double>(config, "TraditionalDetector", "arrow_aspect_min")
            .value_or(2.0);
    detector_config.arrow_aspect_max =
        static_param::get_param<double>(config, "TraditionalDetector", "arrow_aspect_max")
            .value_or(6.0);

    // 准确度阈值
    detector_config.accuracy_threshold =
        static_param::get_param<double>(config, "TraditionalDetector", "accuracy_threshold")
            .value_or(0.8);

    // 调试选项
    detector_config.show_window =
        static_param::get_param<bool>(config, "TraditionalDetector", "show_window")
            .value_or(false);
    detector_config.save_debug_image =
        static_param::get_param<bool>(config, "TraditionalDetector", "save_debug_image")
            .value_or(false);

    debug::print("info", "BuffDetector",
        "TraditionalDetector config: gray_th_red={}, gray_th_blue={}, show={}",
        detector_config.gray_threshold_red,
        detector_config.gray_threshold_blue,
        detector_config.show_window);

    auto detector = std::make_unique<TraditionalDetector>(detector_config);
    detector->set_enemy_color(color);

    return detector;
}

}  // namespace autobuff::detector
