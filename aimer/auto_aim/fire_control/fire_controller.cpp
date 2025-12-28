/**
 * @file fire_controller.cpp
 * @brief 火控主类实现
 */

#include "fire_controller.hpp"

#include <cmath>

#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

FireController::FireController()
{
    gimbal_planner_ = std::make_unique<GimbalPlanner>();
}

FireController::~FireController() = default;

void FireController::reset()
{
    target_selector_.clear_target();
    gimbal_planner_->reset();

    last_selection_ = {};
    last_aim_ = {};
    last_plan_ = {};
    last_spin_aim_ = {};
    lost_count_ = 0;
}

FireCommand FireController::control(
    const predictor::BattlefieldSnapshot& snapshot,
    double current_time,
    const LatencyInfo& latency_in
)
{
    // 读取模式
    use_mpc_ = runtime_param::get_param<bool>("AutoAim.FireControl.use_mpc");

    // 复制延迟信息 (需要迭代更新 fire_to_hit)
    LatencyInfo latency = latency_in;

    // 1. 更新云台状态
    double dt = (last_time_ > 0) ? (current_time - last_time_) : CONTROL_DT;
    gimbal_state_.update(snapshot.self_state.q_imu, dt);
    last_time_ = current_time;

    // 2. 目标选择 (用初始延迟估计)
    double prediction_dt = latency.prediction_latency();
    TargetSelection selection = target_selector_.select(snapshot, prediction_dt);
    last_selection_ = selection;

    // 3. 无目标处理
    if (!selection.has_target || !selection.vehicle) {
        lost_count_++;
        if (lost_count_ > MAX_LOST_COUNT) {
            reset();
        }
        return no_target_command();
    }
    lost_count_ = 0;

    // 4. 迭代更新延迟 (参考 rm.cv.fans filter_to_prediction_time)
    //    用弹道解算后的距离更新 fire_to_hit，然后重新预测位置
    constexpr int NUM_ITERATIONS = 2;
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
        // 预测位置
        prediction_dt = latency.prediction_latency();
        Eigen::Vector3d predicted_pos = selection.vehicle->predict_armor_position(
            selection.armor_idx, prediction_dt
        );

        // 弹道解算
        AimResult aim = trajectory_solver_.solve(
            predicted_pos, snapshot.self_state.bullet_speed
        );

        if (aim.valid) {
            // 用瞄准点距离更新 fire_to_hit
            latency.update_fire_to_hit(aim.distance);
        }
    }

    // 更新选择结果的预测位置 (用最终的延迟)
    selection.predicted_pos = selection.vehicle->predict_armor_position(
        selection.armor_idx, latency.prediction_latency()
    );
    last_selection_ = selection;

    // 5. 分支处理
    if (use_mpc_) {
        return process_mpc(selection, snapshot, latency);
    } else {
        return process_pid(selection, snapshot, latency);
    }
}

FireCommand FireController::process_mpc(
    const TargetSelection& selection,
    const predictor::BattlefieldSnapshot& snapshot,
    const LatencyInfo& latency
)
{
    // MPC 规划 (使用共享的 trajectory_solver_)
    GimbalPlan plan = gimbal_planner_->plan(
        *selection.vehicle,
        gimbal_state_,
        trajectory_solver_,
        latency,
        snapshot.self_state.bullet_speed
    );
    last_plan_ = plan;

    if (!plan.valid) {
        return no_target_command();
    }

    // 弹道解算 (用于开火判断)
    AimResult aim = trajectory_solver_.solve(
        selection.predicted_pos,
        snapshot.self_state.bullet_speed
    );
    last_aim_ = aim;

    if (!aim.valid) {
        return no_target_command();
    }

    // 开火判断 (使用规划后的位置)
    // MPC 模式：使用规划结果作为瞄准角度
    AimResult aim_for_fire = aim;
    aim_for_fire.yaw = plan.yaw;
    aim_for_fire.pitch = plan.pitch;

    bool can_fire = fire_decision_.decide(
        aim_for_fire,
        selection.armor,
        gimbal_state_,
        selection.vehicle->confidence
    );

    return generate_command(selection, plan, aim, can_fire);
}

FireCommand FireController::process_pid(
    const TargetSelection& selection,
    const predictor::BattlefieldSnapshot& snapshot,
    const LatencyInfo& latency
)
{
    // 反陀螺瞄准
    SpinAimResult spin_result = spin_aim_.compute(
        *selection.vehicle,
        latency.now_to_hit()
    );
    last_spin_aim_ = spin_result;

    if (!spin_result.valid) {
        return no_target_command();
    }

    // 弹道解算
    AimResult aim = trajectory_solver_.solve(
        spin_result.target_pos,
        snapshot.self_state.bullet_speed
    );
    last_aim_ = aim;

    if (!aim.valid) {
        return no_target_command();
    }

    // 构造 GimbalPlan (PID 模式无速度/加速度前馈)
    GimbalPlan plan;
    plan.valid = true;
    plan.yaw = aim.yaw;
    plan.pitch = aim.pitch;
    last_plan_ = plan;

    // 获取装甲板 (用于开火判断)
    const predictor::ArmorState* armor = nullptr;
    if (spin_result.armor_idx >= 0 && spin_result.armor_idx < selection.vehicle->armor_count) {
        armor = &selection.vehicle->armors[spin_result.armor_idx];
    }

    // 开火判断
    bool can_fire = fire_decision_.decide(
        aim,
        armor,
        gimbal_state_,
        selection.vehicle->confidence
    );

    // INDIRECT 模式额外检查开火时机
    if (can_fire && spin_result.mode == AimMode::INDIRECT) {
        double fire_advance = runtime_param::get_param<double>(
            "AutoAim.FireControl.PID.fire_advance"
        );
        double now_to_hit = latency.now_to_hit();
        if (spin_result.time_to_fire > now_to_hit + fire_advance) {
            can_fire = false;
        }
    }

    return generate_command(selection, plan, aim, can_fire);
}

FireCommand FireController::generate_command(
    const TargetSelection& selection,
    const GimbalPlan& plan,
    const AimResult& aim,
    bool can_fire
)
{
    FireCommand cmd;
    cmd.control_enabled = true;

    // 云台控制
    cmd.yaw = static_cast<float>(plan.yaw);
    cmd.yaw_vel = static_cast<float>(plan.yaw_vel);
    cmd.yaw_acc = static_cast<float>(plan.yaw_acc);

    cmd.pitch = static_cast<float>(plan.pitch);
    cmd.pitch_vel = static_cast<float>(plan.pitch_vel);
    cmd.pitch_acc = static_cast<float>(plan.pitch_acc);

    // 射击控制
    cmd.allow_fire = true;
    cmd.fire_now = can_fire;

    // 调试信息
    cmd.target_id = selection.target_id;
    cmd.tracking_error = static_cast<float>(
        fire_decision_.compute_tracking_error(aim, gimbal_state_)
    );
    cmd.confidence = static_cast<float>(
        selection.vehicle ? selection.vehicle->confidence : 0
    );

    return cmd;
}

FireCommand FireController::no_target_command()
{
    FireCommand cmd;
    cmd.control_enabled = false;
    cmd.allow_fire = false;
    cmd.fire_now = false;
    cmd.target_id = -1;
    return cmd;
}

}  // namespace autoaim::fire_control
