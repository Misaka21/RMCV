/**
 * @file motion_factory.hpp
 * @brief 运动模型工厂
 *
 * 根据配置创建对应的运动模型实例
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_MOTION_MOTION_FACTORY_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_MOTION_MOTION_FACTORY_HPP__

#include <memory>
#include <string>

#include "motion_interface.hpp"

namespace autoaim::predictor {

/**
 * @brief 运动模型类型枚举
 */
enum class MotionType {
    SPIN,   // SpinMotion (10维, 旧版)
    LMTD,   // LmtdMotion (9维, rm.cv.fans)
    SP      // SpMotion (11维, sp_vision_25)
};

/**
 * @brief 根据枚举创建运动模型
 * @param type 模型类型
 * @param armor_num 装甲板数量 (3 或 4)
 * @return 模型实例的 unique_ptr
 */
std::unique_ptr<MotionInterface> create_motion(MotionType type, int armor_num);

/**
 * @brief 根据配置字符串创建运动模型
 * @param type_str 模型类型字符串 ("spin", "lmtd", "sp")
 * @param armor_num 装甲板数量 (3 或 4)
 * @return 模型实例的 unique_ptr
 *
 * 如果字符串无法识别，默认返回 SpinMotion
 */
std::unique_ptr<MotionInterface> create_motion(const std::string& type_str, int armor_num);

/**
 * @brief 将字符串转换为 MotionType 枚举
 * @param type_str 模型类型字符串
 * @return 对应的 MotionType 枚举值
 */
MotionType motion_type_from_string(const std::string& type_str);

/**
 * @brief 将 MotionType 枚举转换为字符串
 * @param type 模型类型枚举
 * @return 对应的字符串
 */
const char* motion_type_to_string(MotionType type);

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_MOTION_MOTION_FACTORY_HPP__
