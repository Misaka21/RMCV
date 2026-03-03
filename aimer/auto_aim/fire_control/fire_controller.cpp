/**
 * @file fire_controller.cpp
 * @brief 火控主类实现
 */

#include "fire_controller.hpp"

#include <algorithm>
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
    last_solution_frame_id_ = -1;
    last_no_target_frame_id_ = -1;
    last_target_confidence_ = 0.0;
    has_cached_solution_ = false;
    lost_count_ = 0;
    last_fail_stage_ = 0;
    last_time_ = 0.0;
    last_plan_time_ = 0.0;
}

bool FireController::evaluate_fire_window(
    const predictor::BattlefieldSnapshot& snapshot,
    const LatencyInfo& latency,
    double confidence
) const
{
    if (!last_aim_.valid || !last_armor_aim_.valid || !last_selection_.has_target) {
        return false;
    }

    bool can_fire = fire_decision_.decide(
        last_aim_,
        last_armor_aim_,
        gimbal_state_,
        confidence
    );

    // 上层允许开火开关 (来自硬件层注入；不影响瞄准，只影响发射)
    can_fire = can_fire && snapshot.self_state.allow_fire;

    // INDIRECT 模式额外检查开火时机
    if (can_fire && last_armor_aim_.mode == AimMode::INDIRECT) {
        double fire_advance = runtime_param::get_param<double>(
            "AutoAim.FireControl.PID.fire_advance"
        );
        // time_to_fire 是基于图像时刻的预测时间，因此应与 img->hit 同轴比较。
        double img_to_hit = latency.hit_latency();
        if (last_armor_aim_.time_to_fire > img_to_hit + fire_advance) {
            can_fire = false;
        }
    }

    return can_fire;
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

    // 同一帧已经判定无目标，直接返回，避免 500Hz 下重复 select()
    if (!has_cached_solution_ && snapshot.frame_id == last_no_target_frame_id_) {
        last_fail_stage_ = 1;
        return no_target_command();
    }

    // 2. 计算预测时间:
    // 目标状态时间轴在 img 时刻，当前时刻到命中预测应为:
    // (current - img) + send_to_control + fire_to_hit
    // 注意: 这里不应重复叠加 img_to_predict/predict_to_send。
    const double img_age = std::max(0.0, current_time - snapshot.timestamp);
    const double prediction_dt = img_age + latency.send_to_control + latency.fire_to_hit;

    const int prev_target_id = last_selection_.has_target ? last_selection_.target_id : -1;
    const int prev_armor_idx = last_armor_aim_.valid ? last_armor_aim_.armor_idx : -1;

    // 3. 目标选择
    // 同一帧优先沿用上次目标选择，避免 500Hz 下重复 select() 抖动；
    // 但后续仍会用新的 prediction_dt 重算选板与弹道。
    TargetSelection selection;
    if (has_cached_solution_ && snapshot.frame_id == last_solution_frame_id_
        && last_selection_.has_target && snapshot.is_valid(last_selection_.target_id))
    {
        selection = last_selection_;
    } else {
        selection = target_selector_.select(snapshot, gimbal_state_, prediction_dt);
    }
    last_selection_ = selection;

    // 4. 无目标处理
    if (!selection.has_target) {
        last_fail_stage_ = 1;  // 选目标失败
        has_cached_solution_ = false;
        if (snapshot.frame_id != last_no_target_frame_id_) {
            lost_count_++;
            last_no_target_frame_id_ = snapshot.frame_id;
        }
        if (lost_count_ > MAX_LOST_COUNT) {
            reset();
            last_fail_stage_ = 1;
        }
        return no_target_command();
    }
    lost_count_ = 0;
    last_no_target_frame_id_ = -1;

    if (!snapshot.is_valid(selection.target_id)) {
        last_fail_stage_ = 1;
        has_cached_solution_ = false;
        return no_target_command();
    }

    // 获取目标车辆的引用 (用索引访问，避免指针悬空问题)
    const auto& vehicle = snapshot.vehicles[selection.target_id];

    // 更新选择结果的预测位置 (用于调试显示)
    int debug_armor_idx = vehicle.recommended_armor_idx;
    if (debug_armor_idx < 0 || debug_armor_idx >= vehicle.armor_count) {
        debug_armor_idx = 0;
    }
    selection.predicted_pos = vehicle.predict_armor_position(debug_armor_idx, prediction_dt);
    last_selection_ = selection;

    // 5. 装甲板瞄准
    const int preferred_armor_idx = last_armor_aim_.valid ? last_armor_aim_.armor_idx : -1;
    ArmorAimResult armor_result = armor_aim_.compute(
        vehicle,
        prediction_dt,
        &gimbal_state_,
        preferred_armor_idx
    );
    last_armor_aim_ = armor_result;

    if (!armor_result.valid) {
        last_fail_stage_ = 2;  // 装甲板瞄准失败
        has_cached_solution_ = false;
        return no_target_command();
    }

    // 6. 弹道解算
    AimResult aim = ::fire_control::trajectory::solve(
        armor_result.target_pos,
        snapshot.self_state.bullet_speed
    );

    if (!aim.valid) {
        last_fail_stage_ = 3;  // 弹道解算失败
        has_cached_solution_ = false;
        return no_target_command();
    }

    // 7. 构造 GimbalPlan
    const double plan_dt = (last_plan_time_ > 0) ? (current_time - last_plan_time_) : CONTROL_DT;
    double yaw_rate = 0.0;
    double pitch_rate = 0.0;
    if (last_aim_.valid && plan_dt > 1e-4) {
        yaw_rate = GimbalState::normalize_angle(aim.yaw - last_aim_.yaw) / plan_dt;
        pitch_rate = (aim.pitch - last_aim_.pitch) / plan_dt;
    }

    // 目标或装甲板切换时，角速度差分不再连续，避免额外预测出现尖峰
    if (selection.target_id != prev_target_id || armor_result.armor_idx != prev_armor_idx) {
        yaw_rate = 0.0;
        pitch_rate = 0.0;
    }

    GimbalPlan plan;
    plan.valid = true;
    plan.yaw = aim.yaw;
    plan.pitch = aim.pitch;
    plan.yaw_vel = yaw_rate;
    plan.pitch_vel = pitch_rate;
    last_plan_ = plan;
    last_aim_ = aim;
    last_plan_time_ = current_time;

    // 8. 开火判断 (使用 ArmorAimResult 中的装甲板信息)
    last_solution_frame_id_ = snapshot.frame_id;
    last_target_confidence_ = vehicle.confidence;
    has_cached_solution_ = true;
    bool can_fire = evaluate_fire_window(snapshot, latency, vehicle.confidence);

    last_fail_stage_ = 9;  // 成功
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

    const double additional_predict_time = runtime_param::get_param<double>(
        "AutoAim.FireControl.Cmd.additional_predict_time"
    );
    const double max_abs_vel = std::max(0.0, runtime_param::get_param<double>(
        "AutoAim.FireControl.Cmd.max_abs_vel"
    ));

    double yaw_vel_for_ff = plan.yaw_vel;
    double pitch_vel_for_ff = plan.pitch_vel;
    if (max_abs_vel > 0.0) {
        yaw_vel_for_ff = std::clamp(yaw_vel_for_ff, -max_abs_vel, max_abs_vel);
        pitch_vel_for_ff = std::clamp(pitch_vel_for_ff, -max_abs_vel, max_abs_vel);
    }

    // 落点偏置 (运行时可热更新)
    const double aim_offset_yaw = runtime_param::get_param<double>(
        "AutoAim.FireControl.AimOffset.yaw"
    );
    const double aim_offset_pitch = runtime_param::get_param<double>(
        "AutoAim.FireControl.AimOffset.pitch"
    );

    // 云台控制:
    // cmd = 目标角 + 额外预测时间 * 角速度 + 落点偏置
    cmd.yaw = static_cast<float>(plan.yaw + additional_predict_time * yaw_vel_for_ff + aim_offset_yaw);
    cmd.yaw_vel = static_cast<float>(plan.yaw_vel);
    cmd.yaw_acc = static_cast<float>(plan.yaw_acc);

    cmd.pitch = static_cast<float>(
        plan.pitch + additional_predict_time * pitch_vel_for_ff + aim_offset_pitch
    );
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
