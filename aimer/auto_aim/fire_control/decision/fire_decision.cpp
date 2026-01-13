/**
 * @file fire_decision.cpp
 * @brief 开火判断模块实现
 */

#include "fire_decision.hpp"

#include <cmath>

#include "aimer/common/types.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

bool FireDecision::decide(
    const AimResult& aim,
    const ArmorAimResult& armor_aim,
    const GimbalState& gimbal,
    double confidence
) const
{
    // 1. 置信度检查
    double min_confidence = runtime_param::get_param<double>(
        "AutoAim.FireControl.min_confidence"
    );
    if (confidence < min_confidence) {
        return false;
    }

    // 2. 计算角度误差
    double yaw_err = GimbalState::normalize_angle(aim.yaw - gimbal.yaw);
    double pitch_err = aim.pitch - gimbal.pitch;
    double distance = aim.distance;

    // 3. 将角度误差转换为落点偏移距离 (米)
    double hit_offset_yaw = distance * std::tan(std::abs(yaw_err));
    double hit_offset_pitch = distance * std::tan(std::abs(pitch_err));

    // 4. 获取装甲板尺寸 (从 ArmorAimResult)
    double armor_width = (armor_aim.armor_width > 0)
        ? armor_aim.armor_width : SMALL_ARMOR_WIDTH;
    double armor_height = (armor_aim.armor_height > 0)
        ? armor_aim.armor_height : SMALL_ARMOR_HEIGHT;

    // 5. 考虑装甲板朝向 (正对时有效区域最大)
    double cos_inclined = std::abs(std::cos(armor_aim.z_to_v));

    // 6. 开火判断: 落点偏移 < 装甲板有效区域
    double error_rate = runtime_param::get_param<double>(
        "AutoAim.FireControl.error_rate"
    );
    bool yaw_ok = hit_offset_yaw < (armor_width / 2.0) * cos_inclined * error_rate;
    bool pitch_ok = hit_offset_pitch < (armor_height / 2.0) * error_rate;

    return yaw_ok && pitch_ok;
}

double FireDecision::compute_tracking_error(
    const AimResult& aim,
    const GimbalState& gimbal
) const
{
    double yaw_err = GimbalState::normalize_angle(aim.yaw - gimbal.yaw);
    double pitch_err = aim.pitch - gimbal.pitch;
    double distance = aim.distance;

    double hit_offset_yaw = distance * std::tan(std::abs(yaw_err));
    double hit_offset_pitch = distance * std::tan(std::abs(pitch_err));

    return std::hypot(hit_offset_yaw, hit_offset_pitch);
}

}  // namespace autoaim::fire_control
