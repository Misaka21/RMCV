//
// 能量机关检测器工厂 - 根据配置创建检测器
//

#ifndef AUTOBUFF_DETECTOR_FACTORY_HPP
#define AUTOBUFF_DETECTOR_FACTORY_HPP

#include <memory>
#include <string>

#include "common/detector_interface.hpp"

namespace autobuff::detector {

/**
 * @brief 从配置文件创建检测器
 * @param color 初始敌方颜色
 * @param config_file 配置文件名 (相对于 CONFIG_DIR)
 * @return BuffDetectorInterface 接口指针
 *
 * 根据配置文件中的 [Detector.Yolo].backend 选择后端:
 *   - "openvino": OpenVINO 后端
 *   - "tensorrt": TensorRT 后端
 */
std::unique_ptr<BuffDetectorInterface> create_detector_from_config(
    EnemyColor color,
    const std::string& config_file = "buff_detector.toml"
);

}  // namespace autobuff::detector

#endif  // AUTOBUFF_DETECTOR_FACTORY_HPP
