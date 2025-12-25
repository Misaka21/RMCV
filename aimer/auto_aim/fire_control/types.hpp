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

// ==================== 配置结构 ====================

/**
 * @brief 延迟配置
 *
 * 时间轴:
 *   img ─────→ predict ─→ send ─→ control ─→ fire ─→ hit
 *    │                              │          │       │
 *    │<──── img_to_control ────────>│          │       │
 *    │           (需要预测)          │<────────>│       │
 *    │                              │ control   │<────>│
 *    │                              │ _to_fire   fly_time
 *
 * 预测时间 = img_to_control + fly_time
 * 开火时机 = control_to_fire (用于反陀螺打击时机)
 *
 * 参考: rm.cv.fans 延迟模型
 */
struct LatencyConfig {
    // 各阶段延迟 (s)
    double img_to_predict = 0.010;    // 图像传输 + 神经网络推理
    double predict_to_send = 0.002;   // 预测计算 + 发送准备
    double send_to_control = 0.003;   // 通信延迟

    // 总延迟: img → control (用于位置预测)
    double img_to_control() const {
        return img_to_predict + predict_to_send + send_to_control;
    }

    // 控制到开火的延迟 (s)
    // 不参与预测时间计算，仅用于反陀螺打击时机判断
    // @param +: 调大将提早反陀螺模型的打击
    double control_to_fire = 0.020;

    // 稳态偏差补偿时间常数 (s)
    // 电控 PID 跟踪斜坡函数时存在稳态偏差 = yaw_v * t0
    // 需要发送 yaw + t0 * yaw_v 来补偿
    double steady_state_time_constant = 0.015;
};

/**
 * @brief 火控配置
 */
struct FireControlConfig {
    // 延迟配置
    LatencyConfig latency;

    // 射击阈值
    double fire_threshold = 0.02;      // 射击误差阈值 (rad)
    double min_confidence = 0.3;       // 最小置信度

    // MPC 权重
    double q_yaw_pos = 9e6;            // yaw 位置权重
    double q_yaw_vel = 0;              // yaw 速度权重
    double r_yaw_acc = 1.0;            // yaw 控制权重

    double q_pitch_pos = 9e6;          // pitch 位置权重
    double q_pitch_vel = 0;            // pitch 速度权重
    double r_pitch_acc = 1.0;          // pitch 控制权重

    // 加速度约束
    double max_yaw_acc = 50.0;         // yaw 最大加速度 (rad/s²)
    double max_pitch_acc = 100.0;      // pitch 最大加速度 (rad/s²)

    // 弹道参数
    double gravity = 9.8;              // 重力加速度 (m/s²)
    double air_resistance_k = 0.01;    // 空气阻力系数

    // 求解器参数
    int mpc_max_iter = 10;             // MPC 最大迭代次数
    int trajectory_max_iter = 10;      // 弹道求解最大迭代次数
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__
