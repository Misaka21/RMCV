/**
 * @file trajectory_solver.hpp
 * @brief 弹道解算器 - 精确空气阻力模型 + 动打动支持
 *
 * 物理模型: 线性空气阻力 F = -k*m*v (Stokes阻力)
 * 使用 Ceres 非线性优化求解发射角
 *
 * 支持场景:
 *   - 静打静: 双方静止
 *   - 静打动: 目标移动 (需外部预测位置)
 *   - 动打静: 自身移动，需考虑车辆速度
 *   - 动打动: 双方都移动
 *
 * 参数通过 runtime_param::get_param 实时获取
 *
 * 参考: rm.cv.fans (SJTU) ResistanceFuncLinear
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_TRAJECTORY_SOLVER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_TRAJECTORY_SOLVER_HPP__

#include <cmath>

#include <Eigen/Core>

#include "aimer/auto_aim/fire_control/types.hpp"

namespace autoaim::fire_control {

/**
 * @brief 弹道求解输入参数
 */
struct TrajectoryInput {
    // 目标位置 (枪管坐标系，xyz: 前左上)
    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();

    // 弹速
    double bullet_speed = 15.0;

    // 自身车辆速度 (枪管坐标系)
    // 静打动时为零向量，动打动时设置实际速度
    Eigen::Vector3d vehicle_velocity = Eigen::Vector3d::Zero();
};

/**
 * @brief 弹道解算器
 *
 * 使用精确的线性空气阻力模型 + Ceres 优化求解
 *
 * 物理方程 (世界坐标系):
 *   dvx/dt = -k*vx
 *   dvy/dt = -k*vy
 *   dvz/dt = -g - k*vz
 *
 * 解析解:
 *   x(t) = vx0/k * (1 - exp(-k*t))
 *   y(t) = vy0/k * (1 - exp(-k*t))
 *   z(t) = (vz0 + g/k)/k * (1 - exp(-k*t)) - g*t/k
 */
class TrajectorySolver {
public:
    TrajectorySolver() = default;

    /**
     * @brief 解算瞄准角度 (统一接口)
     *
     * 自动根据 vehicle_velocity 选择最优求解方式:
     *   - vehicle_velocity ≈ 0: 使用 2D 求解 (更快)
     *   - vehicle_velocity ≠ 0: 使用 3D 求解 (动打动)
     */
    AimResult solve(const TrajectoryInput& input) const;

    /**
     * @brief 解算瞄准角度 (便捷接口，静打动)
     */
    AimResult solve(
        const Eigen::Vector3d& target_pos,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity = Eigen::Vector3d::Zero()
    ) const;

    /**
     * @brief 估算飞行时间 (快速近似)
     */
    double estimate_fly_time(double distance, double bullet_speed) const;

    /**
     * @brief 计算给定角度的落点 (用于验证)
     */
    Eigen::Vector3d compute_hit_point(
        double yaw,
        double pitch,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity,
        double fly_time
    ) const;

private:
    AimResult solve_2d(const TrajectoryInput& input) const;
    AimResult solve_3d(const TrajectoryInput& input) const;
};

// ============================================================================
// 工具函数
// ============================================================================

/**
 * @brief 计算水平距离
 */
inline double horizontal_distance(const Eigen::Vector3d& pos) {
    return std::hypot(pos.x(), pos.y());
}

/**
 * @brief 计算3D距离
 */
inline double distance_3d(const Eigen::Vector3d& pos) {
    return pos.norm();
}

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_TRAJECTORY_SOLVER_HPP__
