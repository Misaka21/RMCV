/**
 * @file fire_controller.hpp
 * @brief 火控主类
 *
 * 职责:
 *   1. 目标选择 (共享 TargetSelector)
 *   2. 轨迹规划 (MPC 或 PID 模式)
 *   3. 弹道解算 (共享 TrajectorySolver)
 *   4. 开火判断 (共享 FireDecision)
 *
 * 数据流:
 *   BattlefieldSnapshot → 目标选择 → 规划 → 弹道解算 → 开火判断 → FireCommand
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__

#include <memory>

#include "types.hpp"
#include "fire_decision.hpp"
#include "target_selector/target_selector.hpp"
#include "aimer/fire_control/core/trajectory/solver_factory.hpp"
#include "mpc/gimbal_planner.hpp"
#include "pid/spin_aim.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

/**
 * @brief 控制模式
 */
enum class ControlMode {
    MPC,    // MPC 轨迹规划模式 (输出 pos+vel+acc)
    PID     // PID 跟踪模式 (仅输出 pos)
};

/**
 * @brief 火控主类
 *
 * 内部根据 use_mpc 参数选择 MPC 或 PID 模式
 * 共享组件只有一份实例，避免重复创建
 */
class FireController {
public:
    FireController();
    ~FireController();

    // 禁止拷贝
    FireController(const FireController&) = delete;
    FireController& operator=(const FireController&) = delete;

    /**
     * @brief 主控制函数
     *
     * @param snapshot 战场快照 (来自 Predictor)
     * @param current_time 当前时间 (s)
     * @param latency 延迟信息
     * @return 火控指令
     */
    FireCommand control(
        const predictor::BattlefieldSnapshot& snapshot,
        double current_time,
        const LatencyInfo& latency
    );

    /**
     * @brief 重置状态
     */
    void reset();

    /**
     * @brief 获取当前控制模式
     */
    ControlMode current_mode() const;

    // ==================== 调试接口 ====================

    const TargetSelection& last_selection() const { return last_selection_; }
    const AimResult& last_aim() const { return last_aim_; }
    const GimbalPlan& last_plan() const { return last_plan_; }
    const GimbalState& gimbal_state() const { return gimbal_state_; }

private:
    // ==================== MPC 模式处理 ====================

    FireCommand process_mpc(
        const TargetSelection& selection,
        const predictor::BattlefieldSnapshot& snapshot,
        const LatencyInfo& latency
    );

    // ==================== PID 模式处理 ====================

    FireCommand process_pid(
        const TargetSelection& selection,
        const predictor::BattlefieldSnapshot& snapshot,
        const LatencyInfo& latency
    );

    // ==================== 辅助方法 ====================

    FireCommand generate_command(
        const TargetSelection& selection,
        const GimbalPlan& plan,
        const AimResult& aim,
        bool can_fire
    );

    FireCommand no_target_command();

    // ==================== 共享组件 ====================

    TargetSelector target_selector_;
    FireDecision fire_decision_;

    // ==================== MPC 专用 ====================

    std::unique_ptr<GimbalPlanner> gimbal_planner_;

    // ==================== PID 专用 ====================

    SpinAim spin_aim_;

    // ==================== 状态 ====================

    GimbalState gimbal_state_;
    double last_time_ = 0;

    // ==================== 缓存 ====================

    TargetSelection last_selection_;
    AimResult last_aim_;
    GimbalPlan last_plan_;
    SpinAimResult last_spin_aim_;

    int lost_count_ = 0;
    static constexpr int MAX_LOST_COUNT = 30;  // 300ms 后重置
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__
