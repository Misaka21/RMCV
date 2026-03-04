/**
 * @file fire_controller.cpp
 * @brief 火控主类实现
 */

#include "fire_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "aimer/common/math/math.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

namespace {

// rm.cv.fans 中 additional-predict-time 的速度项使用“同一时刻目标角速度”。
// 这里用短时外推 + 二次弹道解算得到局部 aim 角速度，避免跨周期差分抖动。
constexpr double AIM_RATE_DT = 8e-3;  // 8ms

double get_param_or(const std::string& name, double default_value)
{
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (const auto* val = std::get_if<double>(&*ptr)) {
            return *val;
        }
    }
    return default_value;
}

double get_spin_window_rad(const predictor::VehicleState& vehicle)
{
    const double top0_deg = (vehicle.armor_count == 4)
        ? get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors4", 58.8888)
        : get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors_other", 75.0);
    const double top1_deg = get_param_or("AutoAim.FireControl.PID.top1_max_orientation_angle", 0.0);
    const double top2_deg = get_param_or("AutoAim.FireControl.PID.top2_max_orientation_angle", 0.0);

    switch (vehicle.spin.level) {
        case predictor::SpinLevel::HIGH:
            return aimer::math::deg_to_rad(top2_deg);
        case predictor::SpinLevel::LOW:
            return aimer::math::deg_to_rad(top1_deg);
        case predictor::SpinLevel::NONE:
        default:
            return aimer::math::deg_to_rad(top0_deg);
    }
}

double predicted_z_to_v(
    const predictor::VehicleState& vehicle,
    int armor_idx,
    double predict_dt
)
{
    if (armor_idx < 0 || armor_idx >= vehicle.armor_count) {
        return 0.0;
    }

    const auto& armor = vehicle.armors[armor_idx];
    if (!vehicle.spin.active || std::abs(vehicle.spin.omega) < 1e-4) {
        return armor.z_to_v;
    }

    const Eigen::Vector3d pos = vehicle.predict_armor_position(armor_idx, predict_dt);
    const double armor_yaw = armor.yaw + vehicle.spin.omega * predict_dt;
    const double view_yaw = std::atan2(pos.y(), pos.x());
    return aimer::math::reduced_angle(armor_yaw - view_yaw - M_PI);
}

bool estimate_aim_rate_from_target_vel(
    const AimResult& aim_now,
    const ArmorAimResult& armor,
    double bullet_speed,
    double& yaw_rate,
    double& pitch_rate
) {
    yaw_rate = 0.0;
    pitch_rate = 0.0;
    if (!aim_now.valid || bullet_speed <= 1e-3) {
        return false;
    }
    if (!armor.target_vel.allFinite()) {
        return false;
    }

    const Eigen::Vector3d target_pos_next = armor.target_pos + armor.target_vel * AIM_RATE_DT;
    if (!target_pos_next.allFinite() || target_pos_next.squaredNorm() < 1e-9) {
        return false;
    }

    const AimResult aim_next = ::fire_control::trajectory::solve(target_pos_next, bullet_speed);
    if (!aim_next.valid) {
        return false;
    }

    yaw_rate = GimbalState::normalize_angle(aim_next.yaw - aim_now.yaw) / AIM_RATE_DT;
    pitch_rate = (aim_next.pitch - aim_now.pitch) / AIM_RATE_DT;
    return std::isfinite(yaw_rate) && std::isfinite(pitch_rate);
}

}  // namespace

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
    last_prediction_dt_ = 0.0;
    last_time_ = 0.0;

    last_rotate_back_ok_ = true;
    last_rotate_back_active_ = false;
    last_rotate_back_start_ = 0.0;
    last_rotate_back_end_ = 0.0;
    last_rotate_back_command_time_ = 0.0;
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

    return can_fire;
}

bool FireController::evaluate_rotate_back_gate(
    const predictor::VehicleState& vehicle,
    double prediction_dt,
    const LatencyInfo& latency,
    double bullet_speed
)
{
    // 默认通过，只有进入“回转禁发窗口”才阻塞
    last_rotate_back_ok_ = true;
    last_rotate_back_active_ = false;
    last_rotate_back_start_ = 0.0;
    last_rotate_back_end_ = 0.0;
    last_rotate_back_command_time_ = prediction_dt + std::max(0.0, latency.control_to_fire);

    if (!last_aim_.valid || !last_armor_aim_.valid) {
        return true;
    }
    if (!vehicle.spin.active || vehicle.spin.level == predictor::SpinLevel::NONE) {
        return true;
    }
    if (std::abs(vehicle.spin.omega) < 1e-4 || bullet_speed <= 1e-3) {
        return true;
    }

    const int armor_idx = last_armor_aim_.armor_idx;
    if (armor_idx < 0 || armor_idx >= vehicle.armor_count) {
        return true;
    }

    const double control_to_fire = std::max(0.0, latency.control_to_fire);
    if (control_to_fire <= 1e-6) {
        return true;
    }

    // 与 rm.cv.fans 同语义:
    // water_gun_hit = prediction_dt, command_hit = prediction_dt + control_to_fire
    const double time_water_hit = prediction_dt;
    const double time_command_hit = prediction_dt + control_to_fire;
    last_rotate_back_command_time_ = time_command_hit;

    const double omega = vehicle.spin.omega;
    const double armor_yaw_water = vehicle.armors[armor_idx].yaw + omega * time_water_hit;
    const double armor_yaw_command = vehicle.armors[armor_idx].yaw + omega * time_command_hit;
    const double armor_rotate_water_to_command = aimer::math::reduced_angle(
        armor_yaw_command - armor_yaw_water
    );

    // 只有“角速度方向”和“水枪->命中角位移方向”相反，才可能进入回转过程
    if (std::signbit(omega) == std::signbit(armor_rotate_water_to_command)) {
        return true;
    }

    const double max_orientation_angle = get_spin_window_rad(vehicle);
    const double zn_to_armor_water = predicted_z_to_v(vehicle, armor_idx, time_water_hit);
    const double zn_to_rotate_back = (omega > 0.0)
        ? +max_orientation_angle
        : -max_orientation_angle;
    const double armor_water_to_rotate_back = aimer::math::reduced_angle(
        zn_to_rotate_back - zn_to_armor_water
    );

    const double time_start_rotating_back =
        time_water_hit + armor_water_to_rotate_back / omega;
    if (!std::isfinite(time_start_rotating_back) || time_start_rotating_back >= time_command_hit) {
        return true;
    }

    const Eigen::Vector3d pos_when_start =
        vehicle.predict_armor_position(armor_idx, time_start_rotating_back);
    const Eigen::Vector3d pos_when_command =
        vehicle.predict_armor_position(armor_idx, time_command_hit);
    if (!pos_when_start.allFinite() || !pos_when_command.allFinite()) {
        return true;
    }
    if (pos_when_start.squaredNorm() < 1e-9 || pos_when_command.squaredNorm() < 1e-9) {
        return true;
    }

    const AimResult aim_when_start = ::fire_control::trajectory::solve(pos_when_start, bullet_speed);
    const AimResult aim_when_command = ::fire_control::trajectory::solve(pos_when_command, bullet_speed);
    if (!aim_when_start.valid || !aim_when_command.valid) {
        return true;
    }

    const double yaw_barrel_rotate_back = GimbalState::normalize_angle(
        aim_when_command.yaw - aim_when_start.yaw
    );
    const double rotate_time_a = get_param_or("AutoAim.FireControl.PID.angle_to_rotate_time_a", 1.79e-3);
    const double rotate_time_b = get_param_or("AutoAim.FireControl.PID.angle_to_rotate_time_b", 0.093);
    const double rotate_time = rotate_time_a * std::abs(yaw_barrel_rotate_back) * 180.0 / M_PI
        + rotate_time_b;

    const double time_end_rotating_back =
        time_start_rotating_back + std::max(0.0, rotate_time);
    last_rotate_back_active_ = true;
    last_rotate_back_start_ = time_start_rotating_back;
    last_rotate_back_end_ = time_end_rotating_back;

    if (time_start_rotating_back < time_command_hit && time_command_hit < time_end_rotating_back) {
        last_rotate_back_ok_ = false;
    }
    return last_rotate_back_ok_;
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
    last_prediction_dt_ = prediction_dt;

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
    const bool same_target_as_last =
        (prev_target_id >= 0 && selection.target_id == prev_target_id);
    const int preferred_armor_idx =
        (same_target_as_last && prev_armor_idx >= 0) ? prev_armor_idx : -1;
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
    // 使用“同一时刻目标角速度”估计额外预测项，避免跨控制周期差分导致黄圈抖动。
    double yaw_rate = 0.0;
    double pitch_rate = 0.0;
    estimate_aim_rate_from_target_vel(
        aim,
        armor_result,
        snapshot.self_state.bullet_speed,
        yaw_rate,
        pitch_rate
    );

    GimbalPlan plan;
    plan.valid = true;
    plan.yaw = aim.yaw;
    plan.pitch = aim.pitch;
    plan.yaw_vel = yaw_rate;
    plan.pitch_vel = pitch_rate;
    last_plan_ = plan;
    last_aim_ = aim;

    // 8. 开火判断 (使用 ArmorAimResult 中的装甲板信息)
    last_solution_frame_id_ = snapshot.frame_id;
    last_target_confidence_ = vehicle.confidence;
    has_cached_solution_ = true;
    bool can_fire = evaluate_fire_window(snapshot, latency, vehicle.confidence);
    can_fire = can_fire && evaluate_rotate_back_gate(
        vehicle,
        prediction_dt,
        latency,
        snapshot.self_state.bullet_speed
    );

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
