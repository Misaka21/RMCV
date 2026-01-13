/**
 * @file fire_decision.hpp
 * @brief 开火判断模块
 *
 * 物理落点法: 将角度误差转换为落点偏移，与装甲板尺寸比较
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_DECISION_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_DECISION_HPP__

#include "types.hpp"
#include "pid/armor_aim.hpp"

namespace autoaim::fire_control {

/**
 * @brief 开火判断器
 *
 * 判断逻辑:
 *   1. 置信度检查
 *   2. 角度误差 → 落点偏移
 *   3. 落点偏移 < 装甲板有效区域
 */
class FireDecision {
public:
    /**
     * @brief 判断是否可以开火
     *
     * @param aim 弹道解算结果
     * @param armor_aim 装甲板瞄准结果 (包含装甲板尺寸和朝向)
     * @param gimbal 当前云台状态
     * @param confidence 目标置信度
     * @return 是否可以开火
     */
    bool decide(
        const AimResult& aim,
        const ArmorAimResult& armor_aim,
        const GimbalState& gimbal,
        double confidence
    ) const;

    /**
     * @brief 计算跟踪误差 (落点偏移距离)
     */
    double compute_tracking_error(
        const AimResult& aim,
        const GimbalState& gimbal
    ) const;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_DECISION_HPP__
