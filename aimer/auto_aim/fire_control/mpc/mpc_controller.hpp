/**
 * @file mpc_controller.hpp
 * @brief MPC 模式火控策略
 *
 * 特点:
 *   - 输出 pos + vel + acc 前馈
 *   - 使用 TinyMPC 规划平滑轨迹
 *   - 自动处理装甲板切换
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_MPC_MPC_CONTROLLER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_MPC_MPC_CONTROLLER_HPP__

#include <memory>

#include "aimer/auto_aim/fire_control/fire_strategy.hpp"
#include "aimer/auto_aim/fire_control/types.hpp"

namespace autoaim::fire_control {

// 前向声明
class TargetSelector;
class TrajectorySolver;
class GimbalPlanner;

/**
 * @brief MPC 模式火控策略
 *
 * 流程:
 *   1. 目标选择 (TargetSelector)
 *   2. 弹道解算 (TrajectorySolver) - 用于验证
 *   3. MPC 规划 (GimbalPlanner) - 生成轨迹
 *   4. 开火判断 (物理落点法)
 */
class MpcController : public FireStrategy {
public:
    MpcController();
    ~MpcController() override;

    // 禁止拷贝
    MpcController(const MpcController&) = delete;
    MpcController& operator=(const MpcController&) = delete;

    // ==================== FireStrategy 接口 ====================

    FireCommand process(
        const predictor::BattlefieldSnapshot& snapshot,
        double current_time
    ) override;

    void reset() override;

    const char* name() const override { return "MPC"; }

    const TargetSelection& last_selection() const override { return last_selection_; }
    const AimResult& last_aim() const override { return last_aim_; }

    // ==================== MPC 特有接口 ====================

    /**
     * @brief 获取上次 MPC 规划结果
     */
    const GimbalPlan& last_plan() const { return last_plan_; }

private:
    /**
     * @brief 选择目标车辆
     */
    TargetSelection select_target(
        const predictor::BattlefieldSnapshot& snapshot,
        double dt
    );

    /**
     * @brief 弹道解算
     */
    AimResult solve_trajectory(
        const Eigen::Vector3d& target_pos,
        double bullet_speed
    );

    /**
     * @brief MPC 规划
     */
    GimbalPlan plan_gimbal(
        const predictor::VehicleState& target,
        const aimer::RobotState& self_state
    );

    /**
     * @brief 开火条件判断
     */
    bool check_fire_condition(
        const AimResult& aim,
        const TargetSelection& selection
    ) const;

    /**
     * @brief 生成火控指令
     */
    FireCommand generate_command(
        const TargetSelection& selection,
        const GimbalPlan& plan,
        bool fire
    );

    /**
     * @brief 无目标指令
     */
    FireCommand no_target_command();

    /**
     * @brief 从 IMU 四元数提取 yaw/pitch
     */
    static void extract_euler(
        const Eigen::Quaterniond& q,
        double& yaw,
        double& pitch
    );

    // ==================== 子组件 ====================

    std::unique_ptr<TargetSelector> target_selector_;
    std::unique_ptr<TrajectorySolver> trajectory_solver_;
    std::unique_ptr<GimbalPlanner> gimbal_planner_;

    // ==================== 状态缓存 ====================

    TargetSelection last_selection_;
    GimbalPlan last_plan_;
    AimResult last_aim_;
    double last_time_ = 0;

    // 当前云台状态
    double current_yaw_ = 0;
    double current_pitch_ = 0;
    double current_yaw_vel_ = 0;
    double current_pitch_vel_ = 0;

    // 丢失计数
    int lost_count_ = 0;
    static constexpr int MAX_LOST_COUNT = 30;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_MPC_MPC_CONTROLLER_HPP__
