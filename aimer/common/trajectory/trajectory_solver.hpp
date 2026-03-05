/**
 * @file trajectory_solver.hpp
 * @brief 弹道解算器接口 - 独立火控模块
 *
 * 物理模型: 线性空气阻力 F = -k*m*v (Stokes阻力)
 * 使用 Ceres 非线性优化求解发射角
 *
 * 支持场景:
 *   - 静打静: 双方静止
 *   - 静打动: 目标移动 (需外部预测位置)
 *   - 动打静: 自身移动，需考虑车辆速度
 *   - 动打动: 双方都移动
 */

#ifndef __AIMER_FIRE_CONTROL_CORE_TRAJECTORY_SOLVER_HPP__
#define __AIMER_FIRE_CONTROL_CORE_TRAJECTORY_SOLVER_HPP__

#include <cmath>

#include <Eigen/Core>

#include "aimer/common/fire_control_types.hpp"

namespace fire_control {

/**
 * @brief 弹道求解输入参数
 */
struct TrajectoryInput {
    // 目标位置向量（枪口原点 + 世界轴向，xyz: 前左上）
    // 即：target_world - barrel_origin_world
    // 注意：这里不是“枪口轴向坐标系”。
    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();

    // 弹速
    double bullet_speed = 15.0;

    // 自身车辆速度（世界轴向，xyz: 前左上）
    // 静打动时为零向量，动打动时设置实际速度
    Eigen::Vector3d vehicle_velocity = Eigen::Vector3d::Zero();
};

/**
 * @brief 弹道解算器接口
 */
class TrajectorySolverInterface {
public:
    virtual ~TrajectorySolverInterface() = default;

    /**
     * @brief 解算瞄准角度
     */
    virtual AimResult solve(const TrajectoryInput& input) const = 0;

    /**
     * @brief 解算瞄准角度 (便捷接口)
     * @param target_pos 枪口原点 + 世界轴向目标向量
     */
    virtual AimResult solve(
        const Eigen::Vector3d& target_pos,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity = Eigen::Vector3d::Zero()
    ) const = 0;

    /**
     * @brief 估算飞行时间 (快速近似)
     */
    virtual double estimate_fly_time(double distance, double bullet_speed) const = 0;

    /**
     * @brief 计算给定角度的落点 (用于验证)
     */
    virtual Eigen::Vector3d compute_hit_point(
        double yaw,
        double pitch,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity,
        double fly_time
    ) const = 0;
};

/**
 * @brief 线性空气阻力弹道解算器
 *
 * 物理方程 (世界坐标系):
 *   dvx/dt = -k*vx
 *   dvy/dt = -k*vy
 *   dvz/dt = -g - k*vz
 */
class LinearResistanceTrajectorySolver : public TrajectorySolverInterface {
public:
    LinearResistanceTrajectorySolver() = default;

    AimResult solve(const TrajectoryInput& input) const override;

    AimResult solve(
        const Eigen::Vector3d& target_pos,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity = Eigen::Vector3d::Zero()
    ) const override;

    double estimate_fly_time(double distance, double bullet_speed) const override;

    Eigen::Vector3d compute_hit_point(
        double yaw,
        double pitch,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity,
        double fly_time
    ) const override;

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

}  // namespace fire_control

#endif  // __AIMER_FIRE_CONTROL_CORE_TRAJECTORY_SOLVER_HPP__
