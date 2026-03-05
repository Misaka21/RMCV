/**
 * @file fire_decision.cpp
 * @brief 开火判断模块实现
 */

#include "fire_decision.hpp"

#include <cmath>

#include "aimer/common/types.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

FireDecision::DecisionMetrics FireDecision::evaluate(
    const AimResult& aim,
    const ArmorAimResult& armor_aim,
    const GimbalState& gimbal,
    double confidence
) const
{
    DecisionMetrics metrics;

    // 1. 置信度诊断（对齐 rm.cv.fans：开火门控不依赖 predictor 置信度）
    metrics.min_confidence = runtime_param::get_param<double>(
        "AutoAim.FireControl.min_confidence"
    );
    metrics.confidence = confidence;
    metrics.conf_ok = true;

    // 2. 计算角度误差
    double yaw_err = GimbalState::normalize_angle(aim.yaw - gimbal.yaw);
    double pitch_err = aim.pitch - gimbal.pitch;
    double distance = aim.distance;

    // 角误差超过 90° 时，切线函数会翻转符号，直接判定不可开火
    if (std::abs(yaw_err) >= M_PI_2 || std::abs(pitch_err) >= M_PI_2) {
        metrics.angle_ok = false;
        return metrics;
    }
    metrics.angle_ok = true;

    // 3. 将角度误差转换为落点偏移距离 (米)
    metrics.hit_offset_yaw = distance * std::abs(std::tan(yaw_err));
    metrics.hit_offset_pitch = distance * std::abs(std::tan(pitch_err));

    // 4. 获取装甲板尺寸 (从 ArmorAimResult)
    double armor_width = (armor_aim.armor_width > 0)
        ? armor_aim.armor_width : SMALL_ARMOR_WIDTH;
    double armor_height = (armor_aim.armor_height > 0)
        ? armor_aim.armor_height : SMALL_ARMOR_HEIGHT;

    // 5. 考虑装甲板朝向 (正对时有效区域最大)
    double cos_inclined = std::abs(std::cos(armor_aim.z_to_v));

    // 6. 开火判断: 落点偏移 < 装甲板有效区域
    metrics.error_rate = runtime_param::get_param<double>(
        "AutoAim.FireControl.error_rate"
    );
    metrics.yaw_limit = (armor_width / 2.0) * cos_inclined * metrics.error_rate;
    metrics.pitch_limit = (armor_height / 2.0) * metrics.error_rate;
    metrics.yaw_ok = metrics.hit_offset_yaw < metrics.yaw_limit;
    metrics.pitch_ok = metrics.hit_offset_pitch < metrics.pitch_limit;

    return metrics;
}

bool FireDecision::decide(
    const AimResult& aim,
    const ArmorAimResult& armor_aim,
    const GimbalState& gimbal,
    double confidence
) const
{
    return evaluate(aim, armor_aim, gimbal, confidence).pass();
}

double FireDecision::compute_tracking_error(
    const AimResult& aim,
    const GimbalState& gimbal
) const
{
    double yaw_err = GimbalState::normalize_angle(aim.yaw - gimbal.yaw);
    double pitch_err = aim.pitch - gimbal.pitch;
    double distance = aim.distance;

    if (std::abs(yaw_err) >= M_PI_2 || std::abs(pitch_err) >= M_PI_2) {
        return 1e9;
    }

    double hit_offset_yaw = distance * std::abs(std::tan(yaw_err));
    double hit_offset_pitch = distance * std::abs(std::tan(pitch_err));

    return std::hypot(hit_offset_yaw, hit_offset_pitch);
}

}  // namespace autoaim::fire_control
