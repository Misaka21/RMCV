/**
 * @file types.hpp
 * @brief 火控模块通用类型定义
 *
 * 这些类型与具体目标类型无关，是火控模块的基础数据结构
 */

#ifndef __AIMER_FIRE_CONTROL_CORE_TYPES_HPP__
#define __AIMER_FIRE_CONTROL_CORE_TYPES_HPP__

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aimer/common/transformer/transformer.hpp"

namespace fire_control {

// ==================== 常量定义 ====================

constexpr double DEFAULT_CONTROL_DT = 0.002;  // 默认控制周期 2ms (500Hz)

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
     *
     * q_imu 是 IMU 坐标系相对世界坐标系的姿态，需要修正 R_gimbal2imubody
     * 得到真正的 Gimbal 坐标系角度 (与 Barrel/枪管坐标系方向相同)
     */
    void update(const Eigen::Quaterniond& q_imu, double dt) {
        // Gimbal → Imu 的旋转修正
        const auto& R_g2i = aimer::tf::Transform<
            aimer::tf::Frame::Gimbal, aimer::tf::Frame::Imu>::R_;
        // q_gimbal = R_g2i^T * q_imu (从 Imu 坐标系转到 Gimbal 坐标系)
        Eigen::Quaterniond q_gimbal(R_g2i.transpose() * q_imu.toRotationMatrix());

        // 提取 yaw/pitch (ZYX 顺序)
        Eigen::Vector3d euler = q_gimbal.toRotationMatrix().eulerAngles(2, 1, 0);
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
 */
struct LatencyInfo {
    double img_to_predict = 0;     // 图像→预测完成 (直接计算)
    double predict_to_send = 0;    // 预测→发送 (卡尔曼滤波)
    double send_to_control = 0;    // 发送→控制器响应 (静态配置)
    double control_to_fire = 0;    // 控制器→出膛 (静态配置)
    double fire_to_hit = 0;        // 出膛→命中 (TrajectorySolver fly_time)

    // 缓存弹速 (用于 latency_estimator 构建初始值)
    double bullet_speed = 15.0;

    /**
     * @brief 设置 fire_to_hit (使用弹道解算器计算的飞行时间)
     *
     * 注意: fly_time 由 TrajectorySolver 计算，已包含空气阻力等因素，
     * 比简单的 distance/speed 更精确
     */
    void set_fly_time(double fly_time) {
        fire_to_hit = fly_time;
    }

    /**
     * @brief 位置预测延迟 (不含 control_to_fire)
     */
    double prediction_latency() const {
        return img_to_predict + predict_to_send + send_to_control + fire_to_hit;
    }

    /**
     * @brief 命中延迟 (含 control_to_fire)
     */
    double hit_latency() const {
        return img_to_predict + predict_to_send + send_to_control + control_to_fire + fire_to_hit;
    }

    /**
     * @brief 从当前时刻到命中的延迟 (用于开火判断)
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

// ==================== 火控调试信息 ====================

/**
 * @brief 火控调试可视化数据 (供 predictor_node 绘制)
 *
 * 所有角度均为世界坐标系绝对角度 (与 GimbalState 同系)
 * 投影到像素: ypd_to_xyz({yaw, pitch, dist}) → world_to_pixel()
 */
struct FireDebugInfo {
    bool valid = false;
    bool control_enabled = false;
    bool fire_now = false;
    int target_id = -1;
    int armor_idx = -1;

    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();  // 装甲板瞄准位置 (世界坐标)

    // 世界坐标系绝对角度
    double aim_yaw = 0;               // 期望角 (AimResult, 含弹道补偿)
    double aim_pitch = 0;
    double cmd_yaw = 0;               // 发送角 (FireCommand)
    double cmd_pitch = 0;
    double gimbal_yaw = 0;            // 当前云台角
    double gimbal_pitch = 0;

    double distance = 0;
    double tracking_error = 0;
    double fly_time = 0;
    double timestamp = 0;

    // 诊断: 火控线程当前看到的模式 (无论是否 AUTOAIM 都写)
    uint8_t fc_mode = 0;           // 火控线程读到的 aim_mode
    double fc_heartbeat = 0;       // 火控线程每次循环都更新，>0 表示线程存活
    uint16_t snapshot_valid_mask = 0;  // 火控读到的 snapshot.valid_mask
    int snapshot_primary_id = -1;      // 火控读到的 snapshot.primary_target_id
    int snapshot_frame_id = -1;        // 火控读到的 snapshot.frame_id
    int fail_stage = 0;                // 0=未执行, 1=选目标失败, 2=装甲板瞄准失败, 3=弹道解算失败, 9=成功
    double bullet_speed = 0;           // 弹速 (用于诊断弹道解算失败)
};

}  // namespace fire_control

#endif  // __AIMER_FIRE_CONTROL_CORE_TYPES_HPP__
