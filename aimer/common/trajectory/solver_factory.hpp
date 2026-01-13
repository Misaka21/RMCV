/**
 * @file solver_factory.hpp
 * @brief 弹道求解器工厂 - 统一接口选择不同求解器
 */

#ifndef __AIMER_FIRE_CONTROL_CORE_SOLVER_FACTORY_HPP__
#define __AIMER_FIRE_CONTROL_CORE_SOLVER_FACTORY_HPP__

#include "trajectory_solver.hpp"
#include "rk4_trajectory_solver.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace fire_control {

namespace trajectory {

/**
 * @brief 从配置获取求解器类型
 */
inline SolverType get_solver_type_from_config() {
    std::string solver_str = runtime_param::get_param<std::string>(
        "AutoAim.FireControl.Trajectory.solver");

    if (solver_str == "analytic_linear" || solver_str == "ceres") {
        return SolverType::ANALYTIC_LINEAR;
    } else if (solver_str == "rk4_linear") {
        return SolverType::RK4_LINEAR;
    } else if (solver_str == "rk4_quadratic" || solver_str == "rk4") {
        return SolverType::RK4_QUADRATIC;
    }

    return SolverType::ANALYTIC_LINEAR;
}

/**
 * @brief 统一求解接口 (指定求解器类型)
 */
inline AimResult solve(
    const Eigen::Vector3d& target_pos,
    double bullet_speed,
    const Eigen::Vector3d& vehicle_velocity,
    SolverType solver_type)
{
    switch (solver_type) {
        case SolverType::ANALYTIC_LINEAR: {
            static LinearResistanceTrajectorySolver analytic_solver;
            return analytic_solver.solve(target_pos, bullet_speed, vehicle_velocity);
        }

        case SolverType::RK4_LINEAR:
        case SolverType::RK4_QUADRATIC: {
            static Rk4TrajectorySolver rk4_solver;
            return rk4_solver.solve(target_pos, bullet_speed, vehicle_velocity);
        }

        default:
            return AimResult{};
    }
}

/**
 * @brief 统一求解接口 (自动选择求解器)
 */
inline AimResult solve(
    const Eigen::Vector3d& target_pos,
    double bullet_speed,
    const Eigen::Vector3d& vehicle_velocity = Eigen::Vector3d::Zero())
{
    SolverType type = get_solver_type_from_config();
    return solve(target_pos, bullet_speed, vehicle_velocity, type);
}

/**
 * @brief 统一求解接口 (TrajectoryInput)
 */
inline AimResult solve(const TrajectoryInput& input) {
    return solve(input.target_pos, input.bullet_speed, input.vehicle_velocity);
}

/**
 * @brief 统一求解接口 (TrajectoryInput + 指定类型)
 */
inline AimResult solve(const TrajectoryInput& input, SolverType solver_type) {
    return solve(input.target_pos, input.bullet_speed, input.vehicle_velocity, solver_type);
}

/**
 * @brief 获取求解器描述信息
 */
inline const char* get_solver_description(SolverType type) {
    switch (type) {
        case SolverType::ANALYTIC_LINEAR:
            return "解析解+线性阻力 (Ceres优化, 快速, 近距离推荐)";
        case SolverType::RK4_LINEAR:
            return "RK4+线性阻力 (数值积分, 中距离)";
        case SolverType::RK4_QUADRATIC:
            return "RK4+二次阻力 (数值积分, 远距离吊射推荐)";
        default:
            return "未知求解器";
    }
}

}  // namespace trajectory

}  // namespace fire_control

#endif  // __AIMER_FIRE_CONTROL_CORE_SOLVER_FACTORY_HPP__
