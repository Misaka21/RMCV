//
// 检测器工厂 - 根据配置创建检测器
//

#ifndef DETECTOR_FACTORY_HPP
#define DETECTOR_FACTORY_HPP

#include <memory>
#include <stdexcept>
#include <string>

#include "common/detector_interface.hpp"
#include "plugin/param/static_config.hpp"

// 传统检测器 (始终可用)
#include "detector_rv/armor_detector.hpp"

// OpenVINO 检测器 (可选)
#ifdef ENABLE_OPENVINO_DETECTOR
#include "detector_ov/openvino_detector.hpp"
#endif

// TensorRT 检测器 (可选)
#ifdef ENABLE_TENSORRT_DETECTOR
#include "detector_trt/tensorrt_detector.hpp"
#endif

namespace autoaim::detector {

/**
 * @brief YOLO 后端类型
 */
enum class YoloBackend {
    OPENVINO,
    TENSORRT
};

/**
 * @brief 从字符串解析后端类型
 */
inline YoloBackend parse_yolo_backend(const std::string& backend) {
    if (backend == "openvino") {
        return YoloBackend::OPENVINO;
    } else if (backend == "tensorrt") {
        return YoloBackend::TENSORRT;
    } else {
        throw std::runtime_error("Unknown YOLO backend: " + backend);
    }
}

/**
 * @brief 从配置文件创建检测器
 * @param color 初始敌方颜色
 * @param config_file 配置文件名
 * @return DetectorInterface 接口指针
 *
 * 自动根据配置文件中的 [Detector].type 选择检测器类型：
 *   - "traditional": 传统检测器
 *   - "yolo": YOLO 检测器 (根据 backend 选择 OpenVINO/TensorRT)
 */
inline std::unique_ptr<DetectorInterface> create_detector_from_config(
    EnemyColor color,
    const std::string& config_file = "detector.toml"
) {
    auto param = static_param::parse_file(config_file);

    // 读取检测器类型
    std::string type_str = static_param::get_param<std::string>(param, "Detector", "type");

    if (type_str == "traditional") {
        return Detector::from_config(color);
    }
    else if (type_str == "yolo") {
        // 读取后端配置
        std::string backend_str = static_param::get_param<std::string>(
            param, "Detector.yolo", "backend");
        YoloBackend backend = parse_yolo_backend(backend_str);

        switch (backend) {
            case YoloBackend::OPENVINO:
#ifdef ENABLE_OPENVINO_DETECTOR
                return OpenvinoDetector::from_config(color, config_file);
#else
                throw std::runtime_error(
                    "OpenVINO detector not enabled. "
                    "Rebuild with -DENABLE_OPENVINO_DETECTOR=ON"
                );
#endif

            case YoloBackend::TENSORRT:
#ifdef ENABLE_TENSORRT_DETECTOR
                return TensorrtDetector::from_config(color, config_file);
#else
                throw std::runtime_error(
                    "TensorRT detector not enabled. "
                    "Rebuild with -DENABLE_TENSORRT_DETECTOR=ON"
                );
#endif
        }
    }

    throw std::runtime_error("Unknown detector type: " + type_str);
}

} // namespace autoaim::detector

#endif // DETECTOR_FACTORY_HPP
