/**
 * @file fire_decision.hpp
 * @brief 开火判断模块
 *
 * 物理落点法: 将角度误差转换为落点偏移，与装甲板尺寸比较
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_DECISION_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_DECISION_HPP__

#include "aimer/auto_aim/fire_control/types.hpp"
#include "aimer/auto_aim/fire_control/selection/armor_aim.hpp"

namespace autoaim::fire_control {

/**
 * @brief 开火判断器
 *
 * 判断逻辑:
 *   1. 角度误差 → 落点偏移
 *   2. 落点偏移 < 装甲板有效区域
 *
 * 注: predictor 置信度仅用于诊断，不作为开火硬门控（对齐 rm.cv.fans）。
 */
class FireDecision {
public:
    struct DecisionMetrics {
        double min_confidence = 0.0;
        double error_rate = 0.0;
        double confidence = 0.0;

        bool conf_ok = false;
        bool angle_ok = false;
        bool yaw_ok = false;
        bool pitch_ok = false;

        double hit_offset_yaw = 0.0;
        double hit_offset_pitch = 0.0;
        double yaw_limit = 0.0;
        double pitch_limit = 0.0;

        bool pass() const { return conf_ok && angle_ok && yaw_ok && pitch_ok; }
    };

    /**
     * @brief 判断是否可以开火
     *
     * @param aim 弹道解算结果
     * @param armor_aim 装甲板瞄准结果 (包含装甲板尺寸和朝向)
     * @param gimbal 当前云台状态
     * @param confidence 目标置信度 (仅调试显示，不参与开火门控)
     * @return 是否可以开火
     */
    DecisionMetrics evaluate(
        const AimResult& aim,
        const ArmorAimResult& armor_aim,
        const GimbalState& gimbal,
        const Eigen::Quaterniond& q_imu,
        double confidence
    ) const;

    bool decide(
        const AimResult& aim,
        const ArmorAimResult& armor_aim,
        const GimbalState& gimbal,
        const Eigen::Quaterniond& q_imu,
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
