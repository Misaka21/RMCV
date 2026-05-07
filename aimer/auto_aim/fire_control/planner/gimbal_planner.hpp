/**
 * @file gimbal_planner.hpp
 * @brief 云台 MPC 规划器 — 对齐 sp_vision_25 Planner
 *
 * 两种操作模式:
 *   1. 500Hz 实时迭代: step() 每次跑 3-5 次 ADMM, warm-start
 *   2. 参考更新:   snapshot/选板变化时调用 build_reference()
 *
 * 参考轨迹生成: 对未来每个 horizon 步, 预测装甲板位置 + 简易弹道 → 瞄准角
 * MPC 优化: 双积分器 + 加速度盒约束, 平滑轨迹
 *
 * 数据流:
 *   FireController → ArmorAimResult + AimResult + GimbalState
 *     → build_reference() → step() → PlannerOutput → FireCommand
 */

#pragma once

#include <Eigen/Dense>

#include "mpc_solver.hpp"
#include "aimer/auto_aim/fire_control/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/fire_control_types.hpp"

namespace autoaim::fire_control {

// ==================== 配置 ====================

struct PlannerConfig {
    double dt = 0.002;           // 控制周期 (500Hz)
    int horizon = 100;           // 预测步数 (200ms)
    double max_yaw_acc = 50.0;   // rad/s²
    double max_pitch_acc = 100.0;// rad/s²
    double q_pos = 1e6;          // 位置权重
    double q_vel = 1e2;          // 速度权重
    double r = 1.0;              // 控制代价
    double rho = 1.0;            // ADMM 惩罚因子
    int max_iter = 5;            // 每次 solve 的最大迭代数
    int cmd_step = 0;            // 输出的 MPC 解步号 (0 = 当前)
};

// ==================== 规划输出 ====================

struct PlannerOutput {
    bool valid = false;
    double yaw = 0;
    double yaw_vel = 0;
    double yaw_acc = 0;
    double pitch = 0;
    double pitch_vel = 0;
    double pitch_acc = 0;
};

// ==================== 云台规划器 ====================

class GimbalPlanner {
public:
    explicit GimbalPlanner(const PlannerConfig& cfg = {});

    /// 从火控结果生成参考轨迹
    /// @param target        目标状态 (用于 predict_armor_position)
    /// @param armor_idx     选中的装甲板索引
    /// @param img_age       图像延迟 (s, current_time - snapshot.timestamp)
    /// @param hit_offset    从当前到命中的延迟 (send_to_control + fire_to_hit)
    /// @param bullet_speed  弹速 (m/s)
    /// @param self_velocity 自身速度 (world)
    /// @param q_imu         IMU 姿态四元数
    void build_reference(
        const predictor::TargetState& target,
        int armor_idx,
        double img_age,
        double hit_offset,
        double bullet_speed,
        const Eigen::Vector3d& self_velocity,
        const Eigen::Quaterniond& q_imu
    );

    /// 运行一次 MPC 求解, 返回规划输出 (500Hz 调用)
    PlannerOutput step(const GimbalState& gimbal);

    /// 重置内部状态
    void reset();

    // 调试接口
    const Eigen::Matrix2Xd& yaw_reference() const { return yaw_ref_; }
    const Eigen::Matrix2Xd& pitch_reference() const { return pitch_ref_; }
    const DoubleIntegratorMPC& yaw_solver() const { return yaw_mpc_; }
    const DoubleIntegratorMPC& pitch_solver() const { return pitch_mpc_; }

private:
    /// 简易弹道解算 (无阻力抛物线), 返回 (yaw, pitch)
    static Eigen::Vector2d simple_aim(
        const Eigen::Vector3d& armor_pos_world,
        double bullet_speed,
        const Eigen::Quaterniond& q_imu
    );

    PlannerConfig cfg_;
    DoubleIntegratorMPC yaw_mpc_;
    DoubleIntegratorMPC pitch_mpc_;

    // 参考轨迹: 2 × horizon (row 0 = position, row 1 = velocity)
    Eigen::Matrix2Xd yaw_ref_;
    Eigen::Matrix2Xd pitch_ref_;

    bool has_reference_ = false;
    double yaw_unwrap_offset_ = 0;  // yaw 解缠绕累积偏移
};

}  // namespace autoaim::fire_control
