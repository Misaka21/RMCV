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
 * 自动根据配置文件中的 [Detector].type 选择检测器类型：
 *   - "traditional": 传统检测器 (始终可用)
 *   - "yolo": YOLO 检测器 (需要模型文件)
 *
 * 如果请求的后端未编译或模型文件不存在，将抛出运行时异常。
 */
std::unique_ptr<BuffDetectorInterface> create_detector_from_config(
    EnemyColor color,
    const std::string& config_file = "buff_detector.toml"
);

/**
 * @brief 创建传统检测器
 * @param color 初始敌方颜色
 * @param config_file 配置文件名
 * @return BuffDetectorInterface 接口指针
 */
std::unique_ptr<BuffDetectorInterface> create_traditional_detector(
    EnemyColor color,
    const std::string& config_file = "buff_detector.toml"
);

}  // namespace autobuff::detector

#endif  // AUTOBUFF_DETECTOR_FACTORY_HPP
