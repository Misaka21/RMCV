/**
 * @file gimbal_planner.hpp
 * @brief 云台 MPC 轨迹规划器
 *
 * 使用 TinyMPC 求解云台轨迹跟踪问题:
 *   状态: [position, velocity]
 *   控制: acceleration
 *   模型: 双积分器 x[k+1] = A*x[k] + B*u[k]
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_GIMBAL_PLANNER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_GIMBAL_PLANNER_HPP__

#include <memory>

#include <Eigen/Core>

#include "aimer/auto_aim/fire_control/types.hpp"
#include "aimer/auto_aim/fire_control/trajectory/trajectory_solver.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

// 前向声明
namespace tinympc {
    struct TinySolver;
}

namespace autoaim::fire_control {

/**
 * @brief 云台 MPC 规划器
 *
 * 分别对 yaw 和 pitch 轴进行独立的 MPC 规划
 */
class GimbalPlanner {
public:
    struct Config {
        // 时间参数
        double dt = 0.01;              // 控制周期 (s)
        int horizon = 100;             // 预测时域
        int half_horizon = 50;         // 取控制量的位置

        // yaw 轴参数
        double q_yaw_pos = 9e6;        // 位置权重
        double q_yaw_vel = 0;          // 速度权重
        double r_yaw_acc = 1.0;        // 控制权重
        double max_yaw_acc = 50.0;     // 最大加速度 (rad/s²)

        // pitch 轴参数
        double q_pitch_pos = 9e6;
        double q_pitch_vel = 0;
        double r_pitch_acc = 1.0;
        double max_pitch_acc = 100.0;

        // 求解器参数
        int max_iter = 10;             // 最大迭代次数
        double rho = 1.0;              // ADMM 惩罚系数

        // 延迟参数
        double img_to_control = 0.015; // img → control 延迟 (s)
        double steady_state_t0 = 0.015;// 稳态偏差补偿时间常数 (s)

        // 开火决策参数
        double fire_threshold = 0.02;  // 开火误差阈值 (rad)
        double max_armor_angle = 1.2;  // 装甲板最大有效角度 (rad, ~70°)
    };

    explicit GimbalPlanner(const Config& config = {});
    ~GimbalPlanner();

    // 禁止拷贝
    GimbalPlanner(const GimbalPlanner&) = delete;
    GimbalPlanner& operator=(const GimbalPlanner&) = delete;

    /**
     * @brief 规划云台轨迹
     *
     * @param target 目标状态
     * @param current_yaw 当前 yaw (rad)
     * @param current_pitch 当前 pitch (rad)
     * @param current_yaw_vel 当前 yaw 速度 (rad/s)
     * @param current_pitch_vel 当前 pitch 速度 (rad/s)
     * @param bullet_speed 弹速 (m/s)
     * @return 规划结果
     */
    GimbalPlan plan(
        const predictor::VehicleState& target,
        double current_yaw,
        double current_pitch,
        double current_yaw_vel,
        double current_pitch_vel,
        double bullet_speed
    );

    /**
     * @brief 简化规划 (直接跟踪单点)
     */
    GimbalPlan plan_simple(
        const Eigen::Vector3d& target_pos,
        double current_yaw,
        double current_pitch,
        double current_yaw_vel,
        double current_pitch_vel,
        double bullet_speed
    );

    // 配置
    void set_config(const Config& config);
    const Config& config() const { return config_; }

private:
    // MPC 轨迹类型: [yaw, yaw_vel, pitch, pitch_vel] × horizon
    using Trajectory = Eigen::Matrix<double, 4, Eigen::Dynamic>;

    /**
     * @brief 初始化求解器
     */
    void init_solvers();

    /**
     * @brief 生成参考轨迹
     *
     * 同济方案：每个时间步选择最优装甲板，装甲板切换自然出现在轨迹中
     * MPC 在加速度约束下自动规划平滑过渡
     */
    Trajectory generate_reference(
        const predictor::VehicleState& target,
        double yaw0,
        double bullet_speed
    );

    /**
     * @brief 选择某时刻的最优装甲板
     *
     * @param target 目标状态
     * @param t 预测时间 (相对当前)
     * @param out_pos 输出：装甲板位置
     * @return 装甲板索引，-1 表示无有效装甲板
     */
    int select_best_armor_at_time(
        const predictor::VehicleState& target,
        double t,
        Eigen::Vector3d& out_pos
    ) const;

    /**
     * @brief 计算开火误差
     *
     * 考虑开火延迟，计算子弹出膛时刻的预测误差
     */
    void compute_fire_decision(
        GimbalPlan& plan,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& yaw_ref,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& pitch_ref
    ) const;

    /**
     * @brief 求解单轴 MPC
     */
    AxisPlan solve_axis(
        tinympc::TinySolver* solver,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref,
        double pos0,
        double vel0
    );

    /**
     * @brief 角度归一化
     */
    static double normalize_angle(double angle);

    Config config_;

    tinympc::TinySolver* yaw_solver_ = nullptr;
    tinympc::TinySolver* pitch_solver_ = nullptr;

    std::unique_ptr<TrajectorySolver> trajectory_solver_;

    bool initialized_ = false;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_GIMBAL_PLANNER_HPP__
