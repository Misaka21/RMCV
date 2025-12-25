/**
 * @file types.hpp
 * @brief 火控模块类型定义
 *
 * 数据流:
 *   BattlefieldSnapshot → 目标选择 → 弹道解算 → MPC规划 → FireCommand → 串口
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__

#include <Eigen/Core>

#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/robot_state.hpp"

namespace autoaim::fire_control {

// ==================== 常量定义 ====================

constexpr double CONTROL_DT = 0.01;        // 控制周期 10ms (100Hz)
constexpr int MPC_HORIZON = 100;           // MPC 预测时域
constexpr int MPC_HALF_HORIZON = 50;       // 取控制量的位置 (500ms)

// ==================== 瞄准结果 ====================

/**
 * @brief 弹道解算结果
 */
struct AimResult {
    bool valid = false;

    double yaw = 0;            // 目标 yaw (rad)
    double pitch = 0;          // 目标 pitch (rad, 含重力补偿)
    double distance = 0;       // 目标距离 (m)
    double fly_time = 0;       // 子弹飞行时间 (s)

    // 击中点预测
    Eigen::Vector3d hit_point = Eigen::Vector3d::Zero();
};

// ==================== MPC 规划结果 ====================

/**
 * @brief 云台规划结果 (单轴)
 */
struct AxisPlan {
    double position = 0;       // 位置 (rad)
    double velocity = 0;       // 速度 (rad/s)
    double acceleration = 0;   // 加速度 (rad/s²)
};

/**
 * @brief 云台规划结果 (双轴)
 */
struct GimbalPlan {
    bool valid = false;

    AxisPlan yaw;
    AxisPlan pitch;

    double target_yaw = 0;     // 目标 yaw (用于计算误差)
    double target_pitch = 0;   // 目标 pitch

    double tracking_error = 0; // 当前跟踪误差 (rad)
    double fire_error = 0;     // 考虑开火延迟后的预测误差 (rad)
    bool can_fire = false;     // 是否可以开火 (fire_error < threshold)
};

// ==================== 火控指令 ====================

/**
 * @brief 火控输出指令 (发送给下位机)
 */
struct FireCommand {
    bool control_enabled = false;   // 是否启用控制

    // 云台控制 (前馈)
    float yaw = 0;             // yaw 位置 (rad)
    float yaw_vel = 0;         // yaw 速度 (rad/s)
    float yaw_acc = 0;         // yaw 加速度 (rad/s²)

    float pitch = 0;           // pitch 位置 (rad)
    float pitch_vel = 0;       // pitch 速度 (rad/s)
    float pitch_acc = 0;       // pitch 加速度 (rad/s²)

    // 射击控制
    bool allow_fire = false;   // 是否允许射击
    bool fire_now = false;     // 立即射击 (误差足够小)

    // 调试信息
    int target_id = -1;        // 当前目标 ID
    float tracking_error = 0;  // 跟踪误差 (rad)
    float confidence = 0;      // 目标置信度
};

// ==================== 目标选择结果 ====================

/**
 * @brief 目标选择结果
 */
struct TargetSelection {
    bool has_target = false;

    int target_id = -1;
    int armor_idx = -1;

    const predictor::VehicleState* vehicle = nullptr;
    const predictor::ArmorState* armor = nullptr;

    double priority = 0;       // 优先级评分

    // 插值后的位置 (考虑延迟)
    Eigen::Vector3d predicted_pos = Eigen::Vector3d::Zero();
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__
