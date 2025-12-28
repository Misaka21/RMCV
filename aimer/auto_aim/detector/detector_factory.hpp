//
// 检测器工厂 - 根据类型创建检测器
//

#ifndef DETECTOR_FACTORY_HPP
#define DETECTOR_FACTORY_HPP

#include <memory>
#include <stdexcept>

#include "common/detector_interface.hpp"

// 传统检测器 (始终可用)
#include "detector_rv/armor_detector.hpp"

// YOLO检测器 (可选，OpenVINO 后端)
#ifdef ENABLE_YOLO_DETECTOR
#include "detector_ov/openvino_detector.hpp"
#endif

namespace autoaim::detector {

/**
 * @brief 从配置文件创建检测器
 * @param color 初始敌方颜色
 * @param type 检测器类型 (默认传统检测)
 * @return DetectorInterface 接口指针
 */
inline std::unique_ptr<DetectorInterface> create_detector_from_config(
    EnemyColor color,
    DetectorType type = DetectorType::TRADITIONAL
) {
    switch (type) {
        case DetectorType::TRADITIONAL:
            return Detector::from_config(color);

        case DetectorType::YOLO:
#ifdef ENABLE_YOLO_DETECTOR
            return OpenvinoDetector::from_config(color);
#else
            throw std::runtime_error(
                "YOLO detector not enabled. "
                "Rebuild with -DENABLE_YOLO_DETECTOR=ON"
            );
#endif

        default:
            throw std::runtime_error("Unknown detector type");
    }
}

} // namespace autoaim::detector

#endif // DETECTOR_FACTORY_HPP
