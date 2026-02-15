/**
 * @file fire_controller.cpp
 * @brief 火控主类实现
 */

#include "fire_controller.hpp"

#include <cmath>

#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

void FireController::reset()
{
    target_selector_.clear_target();

    last_selection_ = {};
    last_aim_ = {};
    last_plan_ = {};
    last_armor_aim_ = {};
    lost_count_ = 0;
}

FireCommand FireController::control(
    const predictor::BattlefieldSnapshot& snapshot,
    double current_time,
    const LatencyInfo& latency
)
{
    // 注意: latency 已由 fire_control_node.cpp 的 finalize_latency() 完成迭代更新
    //       FireController 直接使用即可

    // 1. 更新云台状态
    double dt = (last_time_ > 0) ? (current_time - last_time_) : CONTROL_DT;
    gimbal_state_.update(snapshot.self_state.q_imu, dt);
    last_time_ = current_time;

    // 1.5 处理预瞄锁定 (右键控制)
    if (snapshot.self_state.aiming_lock) {
        // 右键按下：锁定当前目标
        if (last_selection_.has_target && !target_selector_.is_locked()) {
            target_selector_.force_lock(last_selection_.target_id);
        }
    } else {
        // 右键释放：解除锁定
        if (target_selector_.is_locked()) {
            target_selector_.unlock();
        }
    }

    // 2. 目标选择
    double prediction_dt = latency.prediction_latency();
    TargetSelection selection = target_selector_.select(snapshot, gimbal_state_, prediction_dt);
    last_selection_ = selection;

    // 3. 无目标处理
    if (!selection.has_target) {
        lost_count_++;
        if (lost_count_ > MAX_LOST_COUNT) {
            reset();
        }
        return no_target_command();
    }
    lost_count_ = 0;

    // 获取目标车辆的引用 (用索引访问，避免指针悬空问题)
    const auto& vehicle = snapshot.vehicles[selection.target_id];

    // 更新选择结果的预测位置 (用于调试显示)
    selection.predicted_pos = vehicle.predict_armor_position(
        vehicle.recommended_armor_idx, latency.prediction_latency()
    );
    last_selection_ = selection;

    // 4. 装甲板瞄准
    ArmorAimResult armor_result = armor_aim_.compute(
        vehicle,
        latency.prediction_latency()
    );
    last_armor_aim_ = armor_result;

    if (!armor_result.valid) {
        return no_target_command();
    }

    // 5. 弹道解算
    AimResult aim = ::fire_control::trajectory::solve(
        armor_result.target_pos,
        snapshot.self_state.bullet_speed
    );
    last_aim_ = aim;

    if (!aim.valid) {
        return no_target_command();
    }

    // 6. 构造 GimbalPlan
    GimbalPlan plan;
    plan.valid = true;
    plan.yaw = aim.yaw;
    plan.pitch = aim.pitch;
    last_plan_ = plan;

    // 7. 开火判断 (使用 ArmorAimResult 中的装甲板信息)
    bool can_fire = fire_decision_.decide(
        aim,
        armor_result,
        gimbal_state_,
        vehicle.confidence
    );

    // 上层允许开火开关 (来自硬件层注入；不影响瞄准，只影响发射)
    can_fire = can_fire && snapshot.self_state.allow_fire;

    // INDIRECT 模式额外检查开火时机
    if (can_fire && armor_result.mode == AimMode::INDIRECT) {
        double fire_advance = runtime_param::get_param<double>(
            "AutoAim.FireControl.PID.fire_advance"
        );
        double now_to_hit = latency.now_to_hit();
        if (armor_result.time_to_fire > now_to_hit + fire_advance) {
            can_fire = false;
        }
    }

    return generate_command(selection, plan, aim, can_fire, vehicle.confidence);
}

FireCommand FireController::generate_command(
    const TargetSelection& selection,
    const GimbalPlan& plan,
    const AimResult& aim,
    bool can_fire,
    double confidence
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
    cmd.confidence = static_cast<float>(confidence);

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
