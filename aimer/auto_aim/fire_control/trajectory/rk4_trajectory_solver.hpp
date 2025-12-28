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
 *
 * 对于小口径弹丸 (17mm), 在常见弹速 (15-30m/s) 下,
 * 实际阻力介于线性和二次之间, 但远距离吊射时二次模型更准确
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_RK4_TRAJECTORY_SOLVER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_RK4_TRAJECTORY_SOLVER_HPP__

#include <array>
#include <cmath>
#include <functional>

#include <Eigen/Core>

#include "aimer/auto_aim/fire_control/types.hpp"

namespace autoaim::fire_control {

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
 *
 * 二次阻力物理模型:
 *   F_drag = 0.5 * rho * Cd * A * v^2
 *   a_drag = F_drag / m = 0.5 * rho * Cd * A / m * v^2
 *
 * 令 k = 0.5 * rho * Cd * A / m, 则:
 *   a_drag = k * v^2  (方向与速度相反)
 *
 * 对于 17mm 弹丸:
 *   - 质量 m ≈ 0.041 kg (42g)
 *   - 直径 d = 0.017 m
 *   - 面积 A = π * d^2 / 4 ≈ 2.27e-4 m^2
 *   - 空气密度 rho ≈ 1.225 kg/m^3
 *   - 阻力系数 Cd ≈ 0.47 (球形)
 *   - k ≈ 0.5 * 1.225 * 0.47 * 2.27e-4 / 0.041 ≈ 0.0016 1/m
 *
 * 但实际需要通过标定确定 k 值
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
 *
 * 使用场景:
 *   - 远距离吊射 (> 10m)
 *   - 需要精确考虑空气阻力
 *   - 二次阻力模型
 *
 * 求解流程:
 *   1. 给定目标位置和弹速
 *   2. 使用二分法迭代 pitch 角
 *   3. 每次迭代用 RK4 积分计算落点
 *   4. 收敛后得到最优发射角
 */
class Rk4TrajectorySolver {
public:
    Rk4TrajectorySolver() = default;

    /**
     * @brief 解算瞄准角度 (主接口)
     *
     * @param target_pos 目标位置 (枪管坐标系: x前 y左 z上)
     * @param bullet_speed 弹速 (m/s)
     * @param vehicle_velocity 自身车辆速度 (动打动)
     * @return AimResult 瞄准结果
     */
    AimResult solve(
        const Eigen::Vector3d& target_pos,
        double bullet_speed,
        const Eigen::Vector3d& vehicle_velocity = Eigen::Vector3d::Zero()
    ) const;

    /**
     * @brief 解算瞄准角度 (TrajectoryInput 接口)
     */
    AimResult solve(const TrajectoryInput& input) const;

    /**
     * @brief RK4 积分求弹道
     *
     * @param initial_state 初始状态 [x,y,z,vx,vy,vz]
     * @param target_distance 目标水平距离 (用于终止条件)
     * @param params 阻力参数
     * @return TrajectoryResult 弹道结果
     */
    TrajectoryResult integrate(
        const TrajectoryState& initial_state,
        double target_distance,
        const DragParams& params
    ) const;

    /**
     * @brief 计算落点高度误差 (用于二分法)
     *
     * @param pitch 发射角 (rad)
     * @param yaw 偏航角 (rad)
     * @param bullet_speed 弹速
     * @param vehicle_velocity 车辆速度
     * @param target_pos 目标位置
     * @param params 阻力参数
     * @return 落点高度 - 目标高度 (正值表示打高了)
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
     * @brief 设置积分步长
     */
    void set_dt(double dt) { dt_ = dt; }

    /**
     * @brief 设置最大迭代次数
     */
    void set_max_iterations(int max_iter) { max_iterations_ = max_iter; }

    /**
     * @brief 设置收敛阈值 (米)
     */
    void set_tolerance(double tol) { tolerance_ = tol; }

    /**
     * @brief RK4 单步积分 (公开以供工厂函数使用)
     */
    TrajectoryState rk4_step(
        const TrajectoryState& state,
        double dt,
        const DragParams& params
    ) const;

private:
    /**
     * @brief 弹道微分方程
     *
     * 状态: [x, y, z, vx, vy, vz]
     * 导数: [vx, vy, vz, ax, ay, az]
     *
     * 物理方程:
     *   线性阻力:
     *     ax = -k * vx
     *     ay = -k * vy
     *     az = -g - k * vz
     *
     *   二次阻力:
     *     |v| = sqrt(vx^2 + vy^2 + vz^2)
     *     ax = -k * vx * |v|
     *     ay = -k * vy * |v|
     *     az = -g - k * vz * |v|
     */
    TrajectoryState derivative(
        const TrajectoryState& state,
        const DragParams& params
    ) const;

    double dt_ = 0.001;              // 积分步长 (1ms)
    int max_iterations_ = 50;        // 二分法最大迭代
    double tolerance_ = 0.001;       // 收敛阈值 (1mm)
    double max_fly_time_ = 3.0;      // 最大飞行时间
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

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_RK4_TRAJECTORY_SOLVER_HPP__
