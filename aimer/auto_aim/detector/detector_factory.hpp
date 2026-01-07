//
// 检测器工厂 - 根据配置创建检测器
//
// 工厂接口声明。具体实现在 detector_factory.cpp，
// 通过条件编译链接不同的检测器后端。
//

#ifndef DETECTOR_FACTORY_HPP
#define DETECTOR_FACTORY_HPP

#include <memory>
#include <string>

#include "common/detector_interface.hpp"

namespace autoaim::detector {

/**
 * @brief 从配置文件创建检测器
 * @param color 初始敌方颜色
 * @param config_file 配置文件名
 * @return DetectorInterface 接口指针
 *
 * 自动根据配置文件中的 [Detector].type 选择检测器类型：
 *   - "traditional": 传统检测器 (始终可用)
 *   - "yolo": YOLO 检测器 (根据 backend 选择 OpenVINO/TensorRT)
 *
 * 如果请求的后端未编译，将抛出运行时异常。
 */
std::unique_ptr<DetectorInterface> create_detector_from_config(
    EnemyColor color,
    const std::string& config_file = "detector.toml"
);

}  // namespace autoaim::detector

#endif  // DETECTOR_FACTORY_HPP
