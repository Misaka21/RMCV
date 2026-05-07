/**
 * @file types.hpp
 * @brief AutoAim 火控模块类型定义
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__

#include "aimer/common/fire_control_types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

using ::fire_control::GimbalState;
using ::fire_control::LatencyInfo;
using ::fire_control::AimResult;
using ::fire_control::GimbalPlan;
using ::fire_control::FireCommand;

constexpr double CONTROL_DT = 0.002;

/// 目标选择结果 (debug 用)
struct TargetSelection {
    bool has_target = false;
    int target_id = -1;
    double priority = 0;
    Eigen::Vector3d predicted_pos = Eigen::Vector3d::Zero();
};

/// 瞄准模式
enum class AimMode { DIRECT, INDIRECT };

/// 装甲板瞄准结果
struct ArmorAimResult {
    bool valid = false;
    AimMode mode = AimMode::DIRECT;
    int armor_idx = -1;
    int armor_id = -1;
    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
    double z_to_v = 0;
    double time_to_fire = 0;
    double armor_width = 0;
    double armor_height = 0;
};

/// 开火门控调试
struct FireGateDebug {
    struct TrackingGate {
        double confidence = 0;
        double min_confidence = 0;
        bool conf_ok = true;
        bool angle_ok = false;
        bool yaw_ok = false;
        bool pitch_ok = false;
        double hit_offset_yaw = 0;
        double hit_offset_pitch = 0;
        double yaw_limit = 0;
        double pitch_limit = 0;
        double error_rate = 0;
    };
    TrackingGate tracking;

    // 上层门控
    bool allow_fire_ok = false;

    // 陀螺 swing/out 门控
    bool swing_ok = true;
    bool out_ok = true;
    double swing_error_rate = 0;
    double out_error_rate = 0;
    double swing_offset_yaw = 0;
    double swing_offset_pitch = 0;
    double swing_yaw_limit = 0;
    double swing_pitch_limit = 0;
    double out_offset_yaw = 0;
    double out_offset_pitch = 0;
    double out_yaw_limit = 0;
    double out_pitch_limit = 0;

    bool pass() const {
        return tracking.conf_ok && tracking.angle_ok
            && tracking.yaw_ok && tracking.pitch_ok;
    }
};

}  // namespace autoaim::fire_control

#endif
