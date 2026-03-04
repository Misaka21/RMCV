/**
 * @file fire_controller.hpp
 * @brief 火控主类
 *
 * 职责:
 *   1. 目标选择 (TargetSelector)
 *   2. 装甲板瞄准 (ArmorAim)
 *   3. 弹道解算 (TrajectorySolver)
 *   4. 开火判断 (FireDecision)
 *
 * 数据流:
 *   BattlefieldSnapshot → 目标选择 → 装甲板瞄准 → 弹道解算 → 开火判断 → FireCommand
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__

#include <memory>

#include "types.hpp"
#include "decision/fire_decision.hpp"
#include "selection/target_selector.hpp"
#include "selection/armor_aim.hpp"
#include "aimer/common/trajectory/solver_factory.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

/**
 * @brief 火控主类
 */
class FireController {
public:
    FireController() = default;
    ~FireController() = default;

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

    // ==================== 调试接口 ====================

    const TargetSelection& last_selection() const { return last_selection_; }
    const AimResult& last_aim() const { return last_aim_; }
    const ArmorAimResult& last_armor_aim() const { return last_armor_aim_; }
    const GimbalPlan& last_plan() const { return last_plan_; }
    const GimbalState& gimbal_state() const { return gimbal_state_; }
    int last_fail_stage() const { return last_fail_stage_; }
    double last_prediction_dt() const { return last_prediction_dt_; }
    bool last_rotate_back_ok() const { return last_rotate_back_ok_; }
    bool last_rotate_back_active() const { return last_rotate_back_active_; }
    double last_rotate_back_start() const { return last_rotate_back_start_; }
    double last_rotate_back_end() const { return last_rotate_back_end_; }
    double last_rotate_back_command_time() const { return last_rotate_back_command_time_; }

private:
    // ==================== 辅助方法 ====================

    FireCommand generate_command(
        const TargetSelection& selection,
        const GimbalPlan& plan,
        const AimResult& aim,
        bool can_fire,
        double confidence
    );

    bool evaluate_fire_window(
        const predictor::BattlefieldSnapshot& snapshot,
        const LatencyInfo& latency,
        double confidence
    ) const;

    bool evaluate_rotate_back_gate(
        const predictor::VehicleState& vehicle,
        double prediction_dt,
        const LatencyInfo& latency,
        double bullet_speed,
        const Eigen::Vector3d& self_velocity
    );

    FireCommand no_target_command();

    // ==================== 组件 ====================

    TargetSelector target_selector_;
    FireDecision fire_decision_;
    ArmorAim armor_aim_;

    // ==================== 状态 ====================

    GimbalState gimbal_state_;
    double last_time_ = 0;

    // ==================== 缓存 ====================

    TargetSelection last_selection_;
    AimResult last_aim_;
    GimbalPlan last_plan_;
    ArmorAimResult last_armor_aim_;
    int last_solution_frame_id_ = -1;
    int last_no_target_frame_id_ = -1;
    double last_target_confidence_ = 0.0;
    bool has_cached_solution_ = false;

    int lost_count_ = 0;  // 按“新图像帧”计数，不按 500Hz 控制周期计数
    int last_fail_stage_ = 0;
    double last_prediction_dt_ = 0.0;
    static constexpr int MAX_LOST_COUNT = 60;  // 约 300ms @ 200Hz 新帧率

    // rm.cv.fans: 回转期禁发门控（仅陀螺场景）
    bool last_rotate_back_ok_ = true;
    bool last_rotate_back_active_ = false;
    double last_rotate_back_start_ = 0.0;        // s, 相对 img_t 的预测时间轴
    double last_rotate_back_end_ = 0.0;          // s, 相对 img_t 的预测时间轴
    double last_rotate_back_command_time_ = 0.0; // s, 相对 img_t 的命中时刻
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__
