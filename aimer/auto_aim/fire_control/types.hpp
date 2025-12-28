/**
 * @file types.hpp
 * @brief 火控模块类型定义
 *
 * 数据流:
 *   BattlefieldSnapshot → 目标选择 → 弹道解算 → 规划 → FireCommand
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/robot_state.hpp"

namespace autoaim::fire_control {

// ==================== 常量定义 ====================

constexpr double CONTROL_DT = 0.01;        // 控制周期 10ms (100Hz)

// ==================== 云台状态 ====================

/**
 * @brief 云台当前状态
 */
struct GimbalState {
    double yaw = 0;            // 当前 yaw (rad)
    double pitch = 0;          // 当前 pitch (rad)
    double yaw_vel = 0;        // yaw 角速度 (rad/s)
    double pitch_vel = 0;      // pitch 角速度 (rad/s)

    double last_yaw = 0;
    double last_pitch = 0;

    /**
     * @brief 从 IMU 四元数更新状态
     */
    void update(const Eigen::Quaterniond& q_imu, double dt) {
        // 提取 yaw/pitch (ZYX 顺序)
        Eigen::Vector3d euler = q_imu.toRotationMatrix().eulerAngles(2, 1, 0);
        double new_yaw = euler[0];
        double new_pitch = euler[1];

        // 计算角速度 (简单差分)
        if (dt > 0.001) {
            yaw_vel = normalize_angle(new_yaw - last_yaw) / dt;
            pitch_vel = (new_pitch - last_pitch) / dt;
        }

        last_yaw = yaw;
        last_pitch = pitch;
        yaw = new_yaw;
        pitch = new_pitch;
    }

    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

// ==================== 延迟信息 ====================

/**
 * @brief 延迟信息 (火控全链路延迟)
 *
 * 时间线: img → predict → send → control → fire → hit
 *
 * 用法:
 *   prediction_latency = 位置预测用 (不含 control_to_fire)
 *   hit_latency = 命中时刻计算用 (含 control_to_fire)
 *
 * 注意: fire_to_hit 需要迭代计算
 *   1. 初始用装甲板位置估计
 *   2. 弹道解算后用瞄准点距离更新
 *   3. 重复 2-3 次收敛
 */
struct LatencyInfo {
    double img_to_predict = 0;     // 图像→预测完成 (直接计算)
    double predict_to_send = 0;    // 预测→发送 (卡尔曼滤波)
    double send_to_control = 0;    // 发送→控制器响应 (静态配置)
    double control_to_fire = 0;    // 控制器→出膛 (静态配置)
    double fire_to_hit = 0;        // 出膛→命中 (distance/bullet_speed)

    // 缓存弹速，用于更新 fire_to_hit
    double bullet_speed = 15.0;

    /**
     * @brief 更新 fire_to_hit (用弹道解算后的距离)
     * @param aim_distance 瞄准点距离 (弹道解算输出)
     */
    void update_fire_to_hit(double aim_distance) {
        fire_to_hit = aim_distance / std::max(bullet_speed, 10.0);
    }

    /**
     * @brief 位置预测延迟 (不含 control_to_fire)
     * 参考 rm.cv.fans: 用于预测目标在控制生效时的位置
     */
    double prediction_latency() const {
        return img_to_predict + predict_to_send + send_to_control + fire_to_hit;
    }

    /**
     * @brief 命中延迟 (含 control_to_fire)
     * 用于反陀螺开火时机判断
     */
    double hit_latency() const {
        return img_to_predict + predict_to_send + send_to_control + control_to_fire + fire_to_hit;
    }

    /**
     * @brief 从当前时刻到命中的延迟 (用于开火判断)
     * = send_to_control + control_to_fire + fire_to_hit
     */
    double now_to_hit() const {
        return send_to_control + control_to_fire + fire_to_hit;
    }
};

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

// ==================== 规划结果 ====================

/**
 * @brief 云台规划结果 (MPC 输出)
 */
struct GimbalPlan {
    bool valid = false;

    // 位置
    double yaw = 0;            // yaw (rad)
    double pitch = 0;          // pitch (rad)

    // 速度 (MPC 前馈，PID 模式为 0)
    double yaw_vel = 0;        // yaw 速度 (rad/s)
    double pitch_vel = 0;      // pitch 速度 (rad/s)

    // 加速度 (MPC 前馈，PID 模式为 0)
    double yaw_acc = 0;        // yaw 加速度 (rad/s²)
    double pitch_acc = 0;      // pitch 加速度 (rad/s²)
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
