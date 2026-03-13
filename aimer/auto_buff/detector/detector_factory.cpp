//
// 能量机关检测器工厂实现
//

#include "detector_factory.hpp"

#include <stdexcept>

#include "plugin/param/static_config.hpp"
#include "plugin/debug/logger.hpp"

#ifdef ENABLE_BUFF_OPENVINO_DETECTOR
#include "detector_ov/openvino_buff_detector.hpp"
#endif

#ifdef ENABLE_BUFF_TENSORRT_DETECTOR
#include "detector_trt/tensorrt_buff_detector.hpp"
#endif

namespace autobuff::detector {

std::unique_ptr<BuffDetectorInterface> create_detector_from_config(
    EnemyColor color,
    const std::string& config_file) {

    auto config = static_param::parse_file(config_file);

    // 读取 YOLO 后端类型
    auto backend_str = static_param::get_param<std::string>(
        config, "Detector.Yolo", "backend");
    if (backend_str.empty()) {
        backend_str = "openvino";
    }

    debug::print("info", "BuffDetector",
        "Creating YOLOX rune detector: backend={}", backend_str);

    if (backend_str == "openvino") {
#ifdef ENABLE_BUFF_OPENVINO_DETECTOR
        return OpenvinoBuffDetector::from_config(color, config_file);
#else
        throw std::runtime_error(
            "OpenVINO buff detector requested but not compiled. "
            "Rebuild with -DENABLE_BUFF_OPENVINO_DETECTOR=ON");
#endif
    } else if (backend_str == "tensorrt") {
#ifdef ENABLE_BUFF_TENSORRT_DETECTOR
        return TensorrtBuffDetector::from_config(color, config_file);
#else
        throw std::runtime_error(
            "TensorRT buff detector requested but not compiled. "
            "Rebuild with -DENABLE_BUFF_TENSORRT_DETECTOR=ON");
#endif
    } else {
        throw std::runtime_error(
            "Unknown YOLO backend for buff detector: " + backend_str);
    }
}

}  // namespace autobuff::detector
