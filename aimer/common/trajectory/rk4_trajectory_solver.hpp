/**
 * @file rk4_trajectory_solver.hpp
 * @brief 基于四阶龙格-库塔 (RK4) 的弹道求解器
 *
 * 特点:
 *   - 支持二次空气阻力模型 (适合远距离吊射)
 *   - 数值积分求解, 无需解析解
 *   - 使用二分法迭代求发射角
 *
 * 物理模型:
 *   - 线性阻力: F = -k*m*v           (Stokes阻力, 低速)
 *   - 二次阻力: F = -k*m*v*|v|       (Newton阻力, 高速)
 */

#ifndef __AIMER_FIRE_CONTROL_CORE_RK4_TRAJECTORY_SOLVER_HPP__
#define __AIMER_FIRE_CONTROL_CORE_RK4_TRAJECTORY_SOLVER_HPP__

#include <array>
#include <cmath>

#include <Eigen/Core>

#include "aimer/common/fire_control_types.hpp"
#include "trajectory_solver.hpp"

namespace fire_control {

// ============================================================================
// 空气阻力模型
// ============================================================================

/**
 * @brief 空气阻力模型类型
 */
enum class DragModel {
    LINEAR,     // 线性阻力: a = -k * v
    QUADRATIC,  // 二次阻力: a = -k * v * |v|
};

/**
 * @brief 阻力模型参数
 */
struct DragParams {
    DragModel model = DragModel::QUADRATIC;
    double k = 0.01;     // 阻力系数 (需标定)
    double g = 9.8;      // 重力加速度
};

// ============================================================================
// RK4 弹道状态
// ============================================================================

/**
 * @brief 弹道状态向量 [x, y, z, vx, vy, vz]
 */
using TrajectoryState = std::array<double, 6>;

/**
 * @brief 弹道积分结果
 */
struct TrajectoryResult {
    bool hit = false;           // 是否命中目标平面
    double fly_time = 0;        // 飞行时间
    Eigen::Vector3d hit_point;  // 落点位置
    Eigen::Vector3d hit_velocity;  // 命中时速度
};

// ============================================================================
// RK4 弹道求解器
// ============================================================================

/**
 * @brief 基于 RK4 的弹道求解器
 */
class Rk4TrajectorySolver : public TrajectorySolverInterface {
public:
    Rk4TrajectorySolver() = default;

    /**
     * @brief 解算瞄准角度 (主接口)
     */
    AimResult solve(
        const Eigen::Vector3d& target_pos,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity = Eigen::Vector3d::Zero()
    ) const override;

    AimResult solve(const TrajectoryInput& input) const override;

    double estimate_fly_time(double distance, double bullet_speed) const override;

    Eigen::Vector3d compute_hit_point(
        double yaw,
        double pitch,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity,
        double fly_time
    ) const override;

    /**
     * @brief RK4 积分求弹道
     */
    TrajectoryResult integrate(
        const TrajectoryState& initial_state,
        double target_distance,
        const DragParams& params
    ) const;

    /**
     * @brief 计算落点高度误差 (用于二分法)
     */
    double compute_height_error(
        double pitch,
        double yaw,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity,
        const Eigen::Vector3d& target_pos,
        const DragParams& params
    ) const;

    /**
     * @brief RK4 单步积分
     */
    TrajectoryState rk4_step(
        const TrajectoryState& state,
        double dt,
        const DragParams& params
    ) const;

private:
    TrajectoryState derivative(
        const TrajectoryState& state,
        const DragParams& params
    ) const;

    double dt_ = 0.001;
    int max_iterations_ = 50;
    double tolerance_ = 0.001;
    double max_fly_time_ = 3.0;
};

// ============================================================================
// 弹道求解器工厂
// ============================================================================

/**
 * @brief 求解器类型
 */
enum class SolverType {
    ANALYTIC_LINEAR,   // 解析解 + 线性阻力 (Ceres, 快速)
    RK4_LINEAR,        // RK4 + 线性阻力
    RK4_QUADRATIC,     // RK4 + 二次阻力 (推荐吊射)
};

/**
 * @brief 获取求解器类型名称
 */
inline const char* solver_type_name(SolverType type) {
    switch (type) {
        case SolverType::ANALYTIC_LINEAR: return "analytic_linear";
        case SolverType::RK4_LINEAR: return "rk4_linear";
        case SolverType::RK4_QUADRATIC: return "rk4_quadratic";
        default: return "unknown";
    }
}

}  // namespace fire_control

#endif  // __AIMER_FIRE_CONTROL_CORE_RK4_TRAJECTORY_SOLVER_HPP__
