/**
 * @file fire_controller.hpp
 * @brief 火控主类
 *
 * 职责:
 *   1. 目标选择
 *   2. 弹道解算
 *   3. MPC 轨迹规划
 *   4. 射击决策
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__

#include <memory>
#include <Eigen/Geometry>

#include "aimer/auto_aim/fire_control/types.hpp"
#include "aimer/auto_aim/fire_control/target_selector/target_selector.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

// 前向声明
class TrajectorySolver;
class GimbalPlanner;

/**
 * @brief 火控控制器
 *
 * 从 BattlefieldSnapshot 中选择目标，规划云台轨迹，输出控制指令
 * 参数通过 runtime_param::get_param 实时获取
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
     * @return 火控指令
     */
    FireCommand control(
        const predictor::BattlefieldSnapshot& snapshot,
        double current_time
    );

    /**
     * @brief 重置状态 (丢失目标时调用)
     */
    void reset();

    // 调试接口
    const TargetSelection& last_selection() const { return last_selection_; }
    const GimbalPlan& last_plan() const { return last_plan_; }
    const AimResult& last_aim() const { return last_aim_; }

private:
    /**
     * @brief 阶段1: 目标选择
     */
    TargetSelection select_target(
        const predictor::BattlefieldSnapshot& snapshot,
        double dt
    );

    /**
     * @brief 阶段2: 弹道解算
     */
    AimResult solve_trajectory(
        const Eigen::Vector3d& target_pos,
        double bullet_speed
    );

    /**
     * @brief 阶段3: MPC 规划
     */
    GimbalPlan plan_gimbal(
        const predictor::VehicleState& target,
        const aimer::RobotState& self_state
    );

    /**
     * @brief 开火条件判断 (统一判断逻辑，基于物理落点)
     *
     * 将角度误差转换为落点偏移距离，与装甲板有效区域比较
     */
    bool check_fire_condition(
        const AimResult& aim,
        const TargetSelection& selection
    ) const;

    /**
     * @brief 生成火控指令 (MPC 模式)
     */
    FireCommand generate_command(
        const TargetSelection& selection,
        const GimbalPlan& plan,
        bool fire
    );

    /**
     * @brief 生成简化指令 (仅位置，无 MPC 前馈)
     */
    FireCommand generate_simple_command(
        const TargetSelection& selection,
        const AimResult& aim,
        bool fire
    );

    /**
     * @brief 无目标时的指令
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

    // ==================== 数据 ====================

    std::unique_ptr<TargetSelector> target_selector_;
    std::unique_ptr<TrajectorySolver> trajectory_solver_;
    std::unique_ptr<GimbalPlanner> gimbal_planner_;

    // 状态缓存
    TargetSelection last_selection_;
    GimbalPlan last_plan_;
    AimResult last_aim_;
    double last_time_ = 0;

    // 当前云台状态 (用于 MPC 初始条件)
    double current_yaw_ = 0;
    double current_pitch_ = 0;
    double current_yaw_vel_ = 0;
    double current_pitch_vel_ = 0;

    // 连续丢失计数
    int lost_count_ = 0;
    static constexpr int MAX_LOST_COUNT = 30;  // 300ms 后重置
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__
