//
// 能量机关检测器工厂实现
//

#include "detector_factory.hpp"

#include <stdexcept>

#include "plugin/param/static_config.hpp"
#include "plugin/debug/log.hpp"

#include "traditional/traditional_detector.hpp"

namespace autobuff::detector {

std::unique_ptr<BuffDetectorInterface> create_detector_from_config(
    EnemyColor color,
    const std::string& config_file) {

    // 读取检测器类型
    auto config = static_param::parse_file(config_file);
    auto type_str = static_param::get_param<std::string>(config, "Detector", "type");
    if (type_str.empty()) {
        type_str = "traditional";
    }

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

    // 读取配置参数 (static_param::get_param 返回 T，失败时返回 T{})
    auto gray_red = static_param::get_param<int64_t>(config, "TraditionalDetector", "gray_threshold_red");
    auto gray_blue = static_param::get_param<int64_t>(config, "TraditionalDetector", "gray_threshold_blue");
    detector_config.gray_threshold_red = gray_red != 0 ? static_cast<int>(gray_red) : 80;
    detector_config.gray_threshold_blue = gray_blue != 0 ? static_cast<int>(gray_blue) : 80;

    // R标参数
    auto r_area_min = static_param::get_param<double>(config, "TraditionalDetector", "r_area_min");
    auto r_area_max = static_param::get_param<double>(config, "TraditionalDetector", "r_area_max");
    auto r_circ_min = static_param::get_param<double>(config, "TraditionalDetector", "r_circularity_min");
    auto r_circ_max = static_param::get_param<double>(config, "TraditionalDetector", "r_circularity_max");
    detector_config.r_area_min = r_area_min > 0 ? r_area_min : 100.0;
    detector_config.r_area_max = r_area_max > 0 ? r_area_max : 5000.0;
    detector_config.r_circularity_min = r_circ_min > 0 ? r_circ_min : 0.2;
    detector_config.r_circularity_max = r_circ_max > 0 ? r_circ_max : 0.9;

    // 靶心参数
    auto tgt_area_min = static_param::get_param<double>(config, "TraditionalDetector", "target_area_min");
    auto tgt_area_max = static_param::get_param<double>(config, "TraditionalDetector", "target_area_max");
    auto tgt_asp_min = static_param::get_param<double>(config, "TraditionalDetector", "target_aspect_min");
    auto tgt_asp_max = static_param::get_param<double>(config, "TraditionalDetector", "target_aspect_max");
    detector_config.target_area_min = tgt_area_min > 0 ? tgt_area_min : 60.0;
    detector_config.target_area_max = tgt_area_max > 0 ? tgt_area_max : 6000.0;
    detector_config.target_aspect_min = tgt_asp_min > 0 ? tgt_asp_min : 0.99;
    detector_config.target_aspect_max = tgt_asp_max > 0 ? tgt_asp_max : 1.55;

    // 缺口参数
    auto gap_ratio_min = static_param::get_param<double>(config, "TraditionalDetector", "gap_area_ratio_min");
    auto gap_ratio_max = static_param::get_param<double>(config, "TraditionalDetector", "gap_area_ratio_max");
    auto gap_asp_min = static_param::get_param<double>(config, "TraditionalDetector", "gap_aspect_min");
    auto gap_asp_max = static_param::get_param<double>(config, "TraditionalDetector", "gap_aspect_max");
    detector_config.gap_area_ratio_min = gap_ratio_min > 0 ? gap_ratio_min : 0.025;
    detector_config.gap_area_ratio_max = gap_ratio_max > 0 ? gap_ratio_max : 0.20;
    detector_config.gap_aspect_min = gap_asp_min > 0 ? gap_asp_min : 1.55;
    detector_config.gap_aspect_max = gap_asp_max > 0 ? gap_asp_max : 8.0;

    // 箭头参数
    auto arr_area_min = static_param::get_param<double>(config, "TraditionalDetector", "arrow_area_min");
    auto arr_area_max = static_param::get_param<double>(config, "TraditionalDetector", "arrow_area_max");
    auto arr_asp_min = static_param::get_param<double>(config, "TraditionalDetector", "arrow_aspect_min");
    auto arr_asp_max = static_param::get_param<double>(config, "TraditionalDetector", "arrow_aspect_max");
    detector_config.arrow_area_min = arr_area_min > 0 ? arr_area_min : 100.0;
    detector_config.arrow_area_max = arr_area_max > 0 ? arr_area_max : 4000.0;
    detector_config.arrow_aspect_min = arr_asp_min > 0 ? arr_asp_min : 2.0;
    detector_config.arrow_aspect_max = arr_asp_max > 0 ? arr_asp_max : 6.0;

    // 准确度阈值
    auto accuracy = static_param::get_param<double>(config, "TraditionalDetector", "accuracy_threshold");
    detector_config.accuracy_threshold = accuracy > 0 ? accuracy : 0.8;

    // 调试选项
    detector_config.show_window = static_param::get_param<bool>(config, "TraditionalDetector", "show_window");
    detector_config.save_debug_image = static_param::get_param<bool>(config, "TraditionalDetector", "save_debug_image");

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
