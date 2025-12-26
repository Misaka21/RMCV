/**
 * @file gimbal_planner.hpp
 * @brief 云台 MPC 轨迹规划器
 *
 * 使用 TinyMPC 求解云台轨迹跟踪问题:
 *   状态: [position, velocity]
 *   控制: acceleration
 *   模型: 双积分器 x[k+1] = A*x[k] + B*u[k]
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_MPC_GIMBAL_PLANNER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_MPC_GIMBAL_PLANNER_HPP__

#include <Eigen/Core>

#include "aimer/auto_aim/fire_control/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

// 前向声明
namespace tinympc {
    struct TinySolver;
}

namespace autoaim::fire_control {

// 前向声明
class TrajectorySolver;

/**
 * @brief 云台 MPC 规划器
 *
 * 分别对 yaw 和 pitch 轴进行独立的 MPC 规划
 */
class GimbalPlanner {
public:
    GimbalPlanner();
    ~GimbalPlanner();

    // 禁止拷贝
    GimbalPlanner(const GimbalPlanner&) = delete;
    GimbalPlanner& operator=(const GimbalPlanner&) = delete;

    /**
     * @brief 规划云台轨迹
     *
     * @param target 目标状态
     * @param gimbal 当前云台状态
     * @param solver 弹道解算器 (共享引用)
     * @param latency 延迟信息
     * @param bullet_speed 弹速 (m/s)
     * @return 规划结果
     */
    GimbalPlan plan(
        const predictor::VehicleState& target,
        const GimbalState& gimbal,
        TrajectorySolver& solver,
        const LatencyInfo& latency,
        double bullet_speed
    );

    /**
     * @brief 重置状态
     */
    void reset();

private:
    // MPC 轨迹类型: [yaw, yaw_vel, pitch, pitch_vel] × horizon
    using Trajectory = Eigen::Matrix<double, 4, Eigen::Dynamic>;

    /**
     * @brief 初始化求解器 (如果参数变化则重新初始化)
     */
    void ensure_solvers_initialized();

    /**
     * @brief 生成参考轨迹
     *
     * 同济方案：每个时间步选择最优装甲板，装甲板切换自然出现在轨迹中
     * MPC 在加速度约束下自动规划平滑过渡
     */
    Trajectory generate_reference(
        const predictor::VehicleState& target,
        const GimbalState& gimbal,
        TrajectorySolver& solver,
        const LatencyInfo& latency,
        double bullet_speed
    );

    /**
     * @brief 选择某时刻的最优装甲板
     */
    int select_best_armor_at_time(
        const predictor::VehicleState& target,
        double t,
        Eigen::Vector3d& out_pos
    ) const;

    /**
     * @brief 求解单轴 MPC
     *
     * @return {position, velocity, acceleration}
     */
    std::tuple<double, double, double> solve_axis(
        tinympc::TinySolver* solver,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref,
        double pos0,
        double vel0
    );

    /**
     * @brief 角度归一化
     */
    static double normalize_angle(double angle);

    tinympc::TinySolver* yaw_solver_ = nullptr;
    tinympc::TinySolver* pitch_solver_ = nullptr;

    bool initialized_ = false;

    // 缓存的参数 (用于检测是否需要重新初始化 solver)
    int cached_horizon_ = 0;
    double cached_q_yaw_pos_ = 0;
    double cached_q_pitch_pos_ = 0;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_MPC_GIMBAL_PLANNER_HPP__
