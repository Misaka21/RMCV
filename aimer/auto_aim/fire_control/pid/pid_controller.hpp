/**
 * @file pid_controller.hpp
 * @brief PID 模式火控策略
 *
 * 特点:
 *   - 仅输出位置 (无速度/加速度前馈)
 *   - 内置反陀螺瞄准逻辑 (SpinAim)
 *   - 支持 DIRECT/INDIRECT 两种模式
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_PID_PID_CONTROLLER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_PID_PID_CONTROLLER_HPP__

#include <memory>

#include "aimer/auto_aim/fire_control/fire_strategy.hpp"
#include "aimer/auto_aim/fire_control/types.hpp"
#include "spin_aim.hpp"

namespace autoaim::fire_control {

// 前向声明
class TargetSelector;
class TrajectorySolver;

/**
 * @brief PID 模式火控策略
 *
 * 流程:
 *   1. 目标选择 (TargetSelector)
 *   2. 反陀螺瞄准 (SpinAim) - 选择 DIRECT 或 INDIRECT 模式
 *   3. 弹道解算 (TrajectorySolver)
 *   4. 开火判断 (物理落点法)
 */
class PidController : public FireStrategy {
public:
    PidController();
    ~PidController() override;

    // 禁止拷贝
    PidController(const PidController&) = delete;
    PidController& operator=(const PidController&) = delete;

    // ==================== FireStrategy 接口 ====================

    FireCommand process(
        const predictor::BattlefieldSnapshot& snapshot,
        double current_time
    ) override;

    void reset() override;

    const char* name() const override { return "PID"; }

    const TargetSelection& last_selection() const override { return last_selection_; }
    const AimResult& last_aim() const override { return last_aim_; }

    // ==================== PID 特有接口 ====================

    /**
     * @brief 获取上次反陀螺瞄准结果
     */
    const SpinAimResult& last_spin_aim() const { return last_spin_aim_; }

    /**
     * @brief 获取当前瞄准模式
     */
    AimMode current_aim_mode() const { return last_spin_aim_.mode; }

private:
    /**
     * @brief 选择目标车辆
     */
    TargetSelection select_target(
        const predictor::BattlefieldSnapshot& snapshot,
        double dt
    );

    /**
     * @brief 反陀螺瞄准计算
     */
    SpinAimResult compute_spin_aim(
        const predictor::VehicleState& vehicle,
        double fly_time
    );

    /**
     * @brief 弹道解算
     */
    AimResult solve_trajectory(
        const Eigen::Vector3d& target_pos,
        double bullet_speed
    );

    /**
     * @brief 开火条件判断
     *
     * 将角度误差转换为落点偏移，与装甲板有效区域比较
     */
    bool check_fire_condition(
        const AimResult& aim,
        const SpinAimResult& spin_aim,
        const predictor::ArmorState* armor,
        double confidence
    ) const;

    /**
     * @brief 生成火控指令
     */
    FireCommand generate_command(
        const TargetSelection& selection,
        const SpinAimResult& spin_aim,
        const AimResult& aim,
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
    SpinAim spin_aim_;

    // ==================== 状态缓存 ====================

    TargetSelection last_selection_;
    SpinAimResult last_spin_aim_;
    AimResult last_aim_;
    double last_time_ = 0;

    // 当前云台状态
    double current_yaw_ = 0;
    double current_pitch_ = 0;

    // 丢失计数
    int lost_count_ = 0;
    static constexpr int MAX_LOST_COUNT = 30;  // 300ms 后重置
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_PID_PID_CONTROLLER_HPP__
