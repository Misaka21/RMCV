//
// 检测器工厂 - 实现
//
// 条件编译在这里处理，上层代码不需要知道具体检测器类型。
//

#include "detector_factory.hpp"

#include <stdexcept>

#include "plugin/param/static_config.hpp"

// 传统检测器 (始终可用)
#include "detector_rv/armor_detector.hpp"

// 可选: OpenVINO 检测器
#ifdef ENABLE_OPENVINO_DETECTOR
#include "detector_ov/openvino_detector.hpp"
#endif

// 可选: TensorRT 检测器
#ifdef ENABLE_TENSORRT_DETECTOR
#include "detector_trt/tensorrt_detector.hpp"
#endif

namespace autoaim::detector {

std::unique_ptr<DetectorInterface> create_detector_from_config(
    EnemyColor color,
    const std::string& config_file
) {
    auto param = static_param::parse_file(config_file);

    // 读取检测器类型
    std::string type_str = static_param::get_param<std::string>(param, "Detector", "type");

    if (type_str == "traditional") {
        return Detector::from_config(color);
    }

    if (type_str == "yolo") {
        // 读取后端配置
        std::string backend = static_param::get_param<std::string>(
            param, "Detector.yolo", "backend");

        if (backend == "openvino") {
#ifdef ENABLE_OPENVINO_DETECTOR
            return OpenvinoDetector::from_config(color, config_file);
#else
            throw std::runtime_error(
                "OpenVINO detector not available. "
                "Install OpenVINO and rebuild, or change armor_detector.toml to use another backend.");
#endif
        }

        if (backend == "tensorrt") {
#ifdef ENABLE_TENSORRT_DETECTOR
            return TensorrtDetector::from_config(color, config_file);
#else
            throw std::runtime_error(
                "TensorRT detector not available. "
                "Install TensorRT and rebuild, or change armor_detector.toml to use another backend.");
#endif
        }

        throw std::runtime_error("Unknown YOLO backend: " + backend);
    }

    throw std::runtime_error("Unknown detector type: " + type_str);
}

}  // namespace autoaim::detector
