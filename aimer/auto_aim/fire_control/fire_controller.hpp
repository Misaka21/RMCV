/**
 * @file fire_controller.hpp
 * @brief 火控主类 — 算法驱动，最小状态
 *
 * 完整数据流 (对齐 rm.cv.fans EnemyPredictor):
 *   BattlefieldSnapshot → 选敌 (TargetCatcher)
 *     → 装甲板候选预测 → 瞄准求解 (direct / indirect)
 *     → 弹道解算 → 开火门控 (跟踪 + swing + out + 回转禁发)
 *     → FireCommand
 *
 * 核心算法:
 *   陀螺目标: 预测各板在命中时刻的位置 → direct(窗口内选 swing_cost 最小)
 *     → indirect(选最早入窗板, emerge 到窗口边界)
 *   非陀螺目标: 可见板中选最正对的(z_to_v 最小) → 打中心
 *
 * 开火条件 (纯数学):
 *   1. 跟踪误差 < 装甲板半尺寸 × error_rate
 *   2. 命中时刻 swing 偏移 < 限制 (陀螺)
 *   3. 命中时刻 out 偏移 < 限制 (陀螺)
 *   4. 不在回转禁发窗口内 (陀螺)
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__

#include <memory>

#include "types.hpp"
#include "planner/gimbal_planner.hpp"
#include "aimer/common/trajectory/solver_factory.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

// ==================== 公共工具 ====================

double get_spin_window_rad(const predictor::TargetState& target);

// ==================== 火控主类 ====================

class FireController {
public:
    FireController() = default;
    ~FireController() = default;

    FireController(const FireController&) = delete;
    FireController& operator=(const FireController&) = delete;

    /// 主入口: 输入快照 + 当前时间 + 延迟 → 输出火控指令
    FireCommand control(
        const predictor::BattlefieldSnapshot& snapshot,
        double current_time,
        const LatencyInfo& latency
    );

    void reset();

    // ==================== 调试接口 ====================

    const TargetSelection& last_selection() const { return last_selection_; }
    const AimResult& last_aim() const { return last_aim_; }
    const ArmorAimResult& last_armor_aim() const { return last_armor_aim_; }
    const GimbalPlan& last_plan() const { return last_plan_; }
    const GimbalState& gimbal_state() const { return gimbal_state_; }
    const FireGateDebug& last_gate_debug() const { return last_gate_debug_; }
    int last_fail_stage() const { return last_fail_stage_; }
    double last_prediction_dt() const { return last_prediction_dt_; }
    const LatencyInfo& last_latency() const { return last_latency_; }
    int last_armor_id() const { return last_armor_id_; }
    const PlannerOutput& last_planner_output() const { return last_planner_output_; }
    bool last_rotate_back_ok() const { return last_rotate_back_ok_; }
    bool last_rotate_back_active() const { return last_rotate_back_active_; }
    double last_rotate_back_start() const { return last_rotate_back_start_; }
    double last_rotate_back_end() const { return last_rotate_back_end_; }
    double last_rotate_back_command_time() const { return last_rotate_back_command_time_; }

private:
    // ==================== 选敌 (对齐 rm.cv.fans TargetCatcher) ====================

    struct TargetCatcher {
        int target_id = -1;
        double caught_time = 0;

        void reset() { target_id = -1; caught_time = 0; }

        /// 尝试抓取新目标, 对齐 rm.cv.fans TargetCatcher::try_catch
        void try_catch(int new_id, double current_time, double keep_time);

        /// 获取当前锁存目标, 超时返回 -1
        int get(double current_time, double memorizing_time) const;
    };

    // ==================== 装甲板候选 ====================

    struct ArmorCandidate {
        int idx = -1;
        int id = -1;
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        double z_to_v = 0;
        double radius = 0;
        double z_offset = 0;
        double width = 0;
        double height = 0;
        bool visible = false;
    };

    /// 获取目标所有装甲板在 dt 时刻的候选信息
    static std::vector<ArmorCandidate> get_candidates(
        const predictor::TargetState& target, double dt);

    // ==================== 瞄准求解 ====================

    struct SpinAimProfile {
        double max_orientation_angle = 0;  // rad
        double max_out_error = 0;
        double max_swing_error = 0;
        double max_tracking_error = 0;
        bool allow_indirect = false;
    };

    /// 获取陀螺瞄准配置
    SpinAimProfile get_spin_profile(const predictor::TargetState& target) const;

    /// 瞄准求解: direct(窗口内) → indirect(等待入窗)
    ArmorAimResult solve_armor_aim(
        const predictor::TargetState& target,
        double predict_dt,
        const SpinAimProfile& profile,
        const Eigen::Quaterniond& q_imu,
        double bullet_speed,
        const Eigen::Vector3d& self_velocity,
        int preferred_idx
    );

    /// direct 选板: 窗口内选 swing_cost 最小的
    ArmorAimResult solve_direct(
        const predictor::TargetState& target,
        double predict_dt,
        const std::vector<ArmorCandidate>& candidates,
        double max_orientation_angle,
        const Eigen::Quaterniond& q_imu,
        double bullet_speed,
        const Eigen::Vector3d& self_velocity,
        int preferred_idx
    );

    /// indirect 选板: 选最早进入窗口的, emerge 到窗口边界
    ArmorAimResult solve_indirect(
        const predictor::TargetState& target,
        double predict_dt,
        const std::vector<ArmorCandidate>& candidates,
        double max_orientation_angle,
        double max_out_error,
        const Eigen::Quaterniond& q_imu,
        int preferred_idx
    );

    /// 非陀螺选板: 可见板中选最正对的
    ArmorAimResult solve_non_spin(
        const predictor::TargetState& target,
        double predict_dt,
        const std::vector<ArmorCandidate>& candidates,
        int preferred_idx
    );

    /// 计算 swing cost (枪口需要转动的角度)
    double compute_swing_cost(
        const Eigen::Vector3d& target_pos,
        const Eigen::Quaterniond& q_imu,
        double bullet_speed,
        const Eigen::Vector3d& self_velocity
    ) const;

    // ==================== 弹道解算辅助 ====================

    AimResult solve_trajectory(
        const Eigen::Vector3d& target_pos,
        const Eigen::Quaterniond& q_imu,
        double bullet_speed,
        const Eigen::Vector3d& self_velocity
    ) const;

    // ==================== 开火门控 ====================

    /// 综合开火判断
    bool evaluate_fire_gate(
        const predictor::BattlefieldSnapshot& snapshot,
        const predictor::TargetState& target,
        const LatencyInfo& latency,
        const SpinAimProfile& profile,
        double prediction_dt,
        const Eigen::Vector3d& self_velocity
    );

    /// 回转禁发门控 (对齐 rm.cv.fans)
    bool evaluate_rotate_back_gate(
        const predictor::TargetState& target,
        double prediction_dt,
        const LatencyInfo& latency,
        double bullet_speed,
        const Eigen::Vector3d& self_velocity,
        const Eigen::Quaterniond& q_imu
    );

    // ==================== 指令生成 ====================

    FireCommand no_target_command();

    // ==================== 状态 ====================

    GimbalState gimbal_state_;
    TargetCatcher catcher_;
    GimbalPlanner planner_;
    double last_time_ = 0;

    // ==================== 调试缓存 ====================

    TargetSelection last_selection_;
    AimResult last_aim_;
    ArmorAimResult last_armor_aim_;
    GimbalPlan last_plan_;
    PlannerOutput last_planner_output_;
    FireGateDebug last_gate_debug_;
    LatencyInfo last_latency_;
    double last_prediction_dt_ = 0;
    int last_armor_id_ = -1;
    int last_fail_stage_ = 0;

    // 回转禁发调试
    bool last_rotate_back_ok_ = true;
    bool last_rotate_back_active_ = false;
    double last_rotate_back_start_ = 0;
    double last_rotate_back_end_ = 0;
    double last_rotate_back_command_time_ = 0;
};

}  // namespace autoaim::fire_control

#endif
