/**
 * @file fire_controller.cpp
 * @brief 火控主类实现
 */

#include "fire_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

namespace {

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
            return aimer::math::deg2rad(top2_deg);
        case predictor::SpinLevel::LOW:
            return aimer::math::deg2rad(top1_deg);
        case predictor::SpinLevel::NONE:
        default:
            return aimer::math::deg2rad(top0_deg);
    }
}

double get_spin_out_error(const predictor::VehicleState& vehicle)
{
    const double top0_out = get_param_or("AutoAim.FireControl.PID.top0_max_out_error", 1.8);
    const double top1_out = get_param_or("AutoAim.FireControl.PID.top1_max_out_error", 0.6);
    const double top2_out = get_param_or("AutoAim.FireControl.PID.top2_max_out_error", 1.8);
    switch (vehicle.spin.level) {
        case predictor::SpinLevel::HIGH:
            return std::max(0.0, top2_out);
        case predictor::SpinLevel::LOW:
            return std::max(0.0, top1_out);
        case predictor::SpinLevel::NONE:
        default:
            return std::max(0.0, top0_out);
    }
}

double get_spin_swing_error(const predictor::VehicleState& vehicle)
{
    // 报告语义：max-swing-error 与 max-out-error 解耦。
    // 若未单独配置，回退到同级 max-out-error，保持兼容。
    const double top0_default = get_param_or("AutoAim.FireControl.PID.top0_max_out_error", 1.8);
    const double top1_default = get_param_or("AutoAim.FireControl.PID.top1_max_out_error", 0.6);
    const double top2_default = get_param_or("AutoAim.FireControl.PID.top2_max_out_error", 1.8);
    const double top0_swing = get_param_or("AutoAim.FireControl.PID.top0_max_swing_error", top0_default);
    const double top1_swing = get_param_or("AutoAim.FireControl.PID.top1_max_swing_error", top1_default);
    const double top2_swing = get_param_or("AutoAim.FireControl.PID.top2_max_swing_error", top2_default);
    switch (vehicle.spin.level) {
        case predictor::SpinLevel::HIGH:
            return std::max(0.0, top2_swing);
        case predictor::SpinLevel::LOW:
            return std::max(0.0, top1_swing);
        case predictor::SpinLevel::NONE:
        default:
            return std::max(0.0, top0_swing);
    }
}

bool estimate_aim_rate_from_target_vel(
    const ArmorAimResult& armor,
    const Eigen::Quaterniond& q_imu,
    double& yaw_rate,
    double& pitch_rate
) {
    yaw_rate = 0.0;
    pitch_rate = 0.0;
    if (!armor.valid) {
        return false;
    }
    if (!armor.target_pos.allFinite() || !armor.target_vel.allFinite()) {
        return false;
    }

    // 对齐 rm.cv.fans：additional-predict-time 使用模型同一时刻的 ypd_v，
    // 不做二次弹道外推差分，避免切板时速度方向突变。
    const Eigen::Vector3d rel_pos = aimer::tf::world_to_barrel_origin_world(
        armor.target_pos, q_imu
    );
    const Eigen::Vector3d rel_vel = armor.target_vel;
    if (rel_pos.squaredNorm() < 1e-9 || !rel_pos.allFinite() || !rel_vel.allFinite()) {
        return false;
    }

    const double x = rel_pos.x();
    const double y = rel_pos.y();
    const double z = rel_pos.z();
    const double x_dot = rel_vel.x();
    const double y_dot = rel_vel.y();
    const double z_dot = rel_vel.z();

    const double xy2 = x * x + y * y;
    const double xyz2 = xy2 + z * z;
    const double xy = std::sqrt(xy2);
    if (xy2 < 1e-9 || xyz2 < 1e-9 || xy < 1e-9) {
        return false;
    }

    yaw_rate = -y * x_dot / xy2 + x * y_dot / xy2;
    pitch_rate = z_dot * xy / xyz2 - x * z * x_dot / (xy * xyz2) - y * z * y_dot / (xy * xyz2);
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
    last_gate_debug_ = {};
    last_solution_frame_id_ = -1;
    last_no_target_frame_id_ = -1;
    last_target_confidence_ = 0.0;
    last_latency_ = {};
    last_armor_id_ = -1;
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
    const predictor::VehicleState& vehicle,
    double prediction_dt,
    const Eigen::Vector3d& self_velocity
)
{
    last_gate_debug_ = {};

    if (!last_aim_.valid || !last_armor_aim_.valid || !last_selection_.has_target) {
        return false;
    }

    last_gate_debug_.tracking = fire_decision_.evaluate(
        last_aim_,
        last_armor_aim_,
        gimbal_state_,
        snapshot.self_state.q_imu,
        vehicle.confidence
    );

    bool can_fire = last_gate_debug_.tracking.pass();
    last_gate_debug_.allow_fire_ok = snapshot.self_state.allow_fire;
    can_fire = can_fire && last_gate_debug_.allow_fire_ok;

    // 反陀螺对齐 antitop/rm.cv.fans:
    // 1) 预测控制命中点 tracking_aim（上面已判）
    // 2) 再检查实际出弹命中点 hit_aim 的 swing/out 约束
    if (vehicle.spin.active && std::abs(vehicle.spin.omega) > 1e-4) {
        const double control_to_fire = std::max(0.0, latency.control_to_fire);
        const double hit_dt = prediction_dt + control_to_fire;

        ArmorAimResult hit_armor = armor_aim_.compute(
            vehicle,
            hit_dt,
            &gimbal_state_,
            &snapshot.self_state.q_imu,
            -1
        );
        if (!hit_armor.valid) {
            last_gate_debug_.swing_ok = false;
            last_gate_debug_.out_ok = false;
            return false;
        }

        const Eigen::Vector3d hit_target_vec = aimer::tf::world_to_barrel_origin_world(
            hit_armor.target_pos, snapshot.self_state.q_imu
        );
        const AimResult hit_aim = ::fire_control::trajectory::solve(
            hit_target_vec,
            snapshot.self_state.bullet_speed,
            self_velocity
        );
        if (!hit_aim.valid) {
            last_gate_debug_.swing_ok = false;
            last_gate_debug_.out_ok = false;
            return false;
        }

        const double swing_error = get_spin_swing_error(vehicle);
        const double out_error = get_spin_out_error(vehicle);
        last_gate_debug_.swing_error_rate = swing_error;
        last_gate_debug_.out_error_rate = out_error;

        auto calc_gate = [](
            const AimResult& aim,
            const GimbalState& gimbal,
            double armor_width,
            double armor_height,
            double z_to_v,
            double error_rate,
            double& out_offset_yaw,
            double& out_offset_pitch,
            double& out_limit_yaw,
            double& out_limit_pitch,
            bool& out_yaw_ok,
            bool& out_pitch_ok
        ) {
            out_offset_yaw = 0.0;
            out_offset_pitch = 0.0;
            out_limit_yaw = 0.0;
            out_limit_pitch = 0.0;
            out_yaw_ok = false;
            out_pitch_ok = false;

            const double yaw_err = GimbalState::normalize_angle(aim.yaw - gimbal.yaw);
            const double pitch_err = aim.pitch - gimbal.pitch;
            if (std::abs(yaw_err) >= M_PI_2 || std::abs(pitch_err) >= M_PI_2) {
                return;
            }

            out_offset_yaw = aim.distance * std::abs(std::tan(yaw_err));
            out_offset_pitch = aim.distance * std::abs(std::tan(pitch_err));
            const double cos_inclined = std::abs(std::cos(z_to_v));
            out_limit_yaw = (armor_width * 0.5) * cos_inclined * error_rate;
            out_limit_pitch = (armor_height * 0.5) * error_rate;
            out_yaw_ok = out_offset_yaw < out_limit_yaw;
            out_pitch_ok = out_offset_pitch < out_limit_pitch;
        };

        bool swing_yaw_ok = false;
        bool swing_pitch_ok = false;
        calc_gate(
            hit_aim,
            gimbal_state_,
            hit_armor.armor_width,
            hit_armor.armor_height,
            hit_armor.z_to_v,
            swing_error,
            last_gate_debug_.swing_offset_yaw,
            last_gate_debug_.swing_offset_pitch,
            last_gate_debug_.swing_yaw_limit,
            last_gate_debug_.swing_pitch_limit,
            swing_yaw_ok,
            swing_pitch_ok
        );
        last_gate_debug_.swing_ok = swing_yaw_ok && swing_pitch_ok;

        // out gate: emerging 瞄点 与 同时刻装甲板中心 的差异不能过大
        const Eigen::Vector3d hit_center = vehicle.predict_armor_position(hit_armor.armor_idx, hit_dt);
        const Eigen::Vector3d hit_center_vec = aimer::tf::world_to_barrel_origin_world(
            hit_center, snapshot.self_state.q_imu
        );
        const AimResult hit_center_aim = ::fire_control::trajectory::solve(
            hit_center_vec,
            snapshot.self_state.bullet_speed,
            self_velocity
        );
        if (!hit_center_aim.valid) {
            last_gate_debug_.out_ok = false;
        } else {
            const double dyaw = GimbalState::normalize_angle(hit_aim.yaw - hit_center_aim.yaw);
            const double dpitch = hit_aim.pitch - hit_center_aim.pitch;
            if (std::abs(dyaw) >= M_PI_2 || std::abs(dpitch) >= M_PI_2) {
                last_gate_debug_.out_ok = false;
            } else {
                const double ref_dist = std::max(1e-3, hit_center_aim.distance);
                last_gate_debug_.out_offset_yaw = ref_dist * std::abs(std::tan(dyaw));
                last_gate_debug_.out_offset_pitch = ref_dist * std::abs(std::tan(dpitch));
                const double cos_inclined = std::abs(std::cos(hit_armor.z_to_v));
                last_gate_debug_.out_yaw_limit =
                    (hit_armor.armor_width * 0.5) * cos_inclined * out_error;
                last_gate_debug_.out_pitch_limit =
                    (hit_armor.armor_height * 0.5) * out_error;
                const bool out_yaw_ok = last_gate_debug_.out_offset_yaw < last_gate_debug_.out_yaw_limit;
                const bool out_pitch_ok = last_gate_debug_.out_offset_pitch < last_gate_debug_.out_pitch_limit;
                last_gate_debug_.out_ok = out_yaw_ok && out_pitch_ok;
            }
        }

        can_fire = can_fire && last_gate_debug_.swing_ok && last_gate_debug_.out_ok;
    } else {
        last_gate_debug_.swing_ok = true;
        last_gate_debug_.out_ok = true;
    }

    return can_fire;
}

bool FireController::evaluate_rotate_back_gate(
    const predictor::VehicleState& vehicle,
    double prediction_dt,
    const LatencyInfo& latency,
    double bullet_speed,
    const Eigen::Vector3d& self_velocity,
    const Eigen::Quaterniond& q_imu
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
    const double control_to_fire = std::max(0.0, latency.control_to_fire);
    if (control_to_fire <= 1e-6) {
        return true;
    }

    // 对齐 rm.cv.fans: 回转门控需要比较两个时刻“被选中状态”的角位移方向，
    // 不是同一块板在短时间内的自旋角位移。
    const double time_water_hit = prediction_dt;
    const double time_command_hit = prediction_dt + control_to_fire;
    last_rotate_back_command_time_ = time_command_hit;

    ArmorAimResult water_aim = armor_aim_.compute(
        vehicle, time_water_hit, &gimbal_state_, &q_imu, -1
    );
    if (!water_aim.valid || water_aim.armor_idx < 0 || water_aim.armor_idx >= vehicle.armor_count) {
        return true;
    }

    ArmorAimResult command_aim = armor_aim_.compute(
        vehicle, time_command_hit, &gimbal_state_, &q_imu, -1
    );
    if (!command_aim.valid || command_aim.armor_idx < 0 || command_aim.armor_idx >= vehicle.armor_count) {
        return true;
    }

    auto armor_yaw_at = [&](int armor_idx, double t) {
        return vehicle.armors[armor_idx].yaw + vehicle.spin.omega * t;
    };

    const double omega = vehicle.spin.omega;
    const double armor_yaw_water = armor_yaw_at(water_aim.armor_idx, time_water_hit);
    const double armor_yaw_command = armor_yaw_at(command_aim.armor_idx, time_command_hit);
    const double armor_rotate_water_to_command = aimer::math::reduced_angle(
        armor_yaw_command - armor_yaw_water
    );

    // 只有“角速度方向”和“水枪->命中角位移方向”相反，才可能进入回转过程
    if (std::signbit(omega) == std::signbit(armor_rotate_water_to_command)) {
        return true;
    }

    const double max_orientation_angle = get_spin_window_rad(vehicle);
    const double zn_to_armor_water = water_aim.z_to_v;
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
        vehicle.predict_armor_position(water_aim.armor_idx, time_start_rotating_back);
    if (!pos_when_start.allFinite() || !command_aim.target_pos.allFinite()) {
        return true;
    }
    if (pos_when_start.squaredNorm() < 1e-9 || command_aim.target_pos.squaredNorm() < 1e-9) {
        return true;
    }

    const Eigen::Vector3d pos_when_start_vec = aimer::tf::world_to_barrel_origin_world(
        pos_when_start, q_imu
    );
    const Eigen::Vector3d command_pos_vec = aimer::tf::world_to_barrel_origin_world(
        command_aim.target_pos, q_imu
    );
    const AimResult aim_when_start = ::fire_control::trajectory::solve(
        pos_when_start_vec, bullet_speed, self_velocity
    );
    const AimResult aim_when_command = ::fire_control::trajectory::solve(
        command_pos_vec, bullet_speed, self_velocity
    );
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

bool FireController::solve_aim_with_latency_iteration(
    const predictor::BattlefieldSnapshot& snapshot,
    const predictor::VehicleState& vehicle,
    const Eigen::Vector3d& self_velocity,
    double current_time,
    const LatencyInfo& base_latency,
    int preferred_armor_idx,
    ArmorAimResult& armor_result,
    AimResult& aim,
    GimbalPlan& plan,
    LatencyInfo& out_latency
)
{
    out_latency = base_latency;
    const double img_age = std::max(0.0, current_time - snapshot.timestamp);
    const int iter_count = static_cast<int>(std::clamp(
        get_param_or("AutoAim.FireControl.Latency.iterations", 2.0), 1.0, 5.0
    ));

    auto solve_once = [&](
        double prediction_dt,
        int preferred_idx,
        ArmorAimResult& out_armor,
        AimResult& out_aim
    ) -> bool {
        out_armor = armor_aim_.compute(
            vehicle,
            prediction_dt,
            &gimbal_state_,
            &snapshot.self_state.q_imu,
            preferred_idx
        );
        if (!out_armor.valid) {
            return false;
        }

        const Eigen::Vector3d target_vec = aimer::tf::world_to_barrel_origin_world(
            out_armor.target_pos, snapshot.self_state.q_imu
        );
        out_aim = ::fire_control::trajectory::solve(
            target_vec,
            snapshot.self_state.bullet_speed,
            self_velocity
        );
        return out_aim.valid;
    };

    int iter_preferred_idx = preferred_armor_idx;
    for (int i = 0; i < iter_count; ++i) {
        const double prediction_dt =
            img_age + out_latency.send_to_control + out_latency.fire_to_hit;

        ArmorAimResult iter_armor;
        AimResult iter_aim;
        if (!solve_once(prediction_dt, iter_preferred_idx, iter_armor, iter_aim)) {
            return false;
        }

        if (std::isfinite(iter_aim.fly_time) && iter_aim.fly_time > 0.0) {
            out_latency.set_fly_time(iter_aim.fly_time);
        }
        if (iter_armor.armor_idx >= 0 && iter_armor.armor_idx < vehicle.armor_count) {
            iter_preferred_idx = iter_armor.armor_idx;
        }
    }

    const double final_prediction_dt =
        img_age + out_latency.send_to_control + out_latency.fire_to_hit;
    if (!solve_once(final_prediction_dt, iter_preferred_idx, armor_result, aim)) {
        return false;
    }

    double yaw_rate = 0.0;
    double pitch_rate = 0.0;
    estimate_aim_rate_from_target_vel(
        armor_result,
        snapshot.self_state.q_imu,
        yaw_rate,
        pitch_rate
    );

    plan = {};
    plan.valid = true;
    plan.yaw = aim.yaw;
    plan.pitch = aim.pitch;
    plan.yaw_vel = yaw_rate;
    plan.pitch_vel = pitch_rate;
    last_prediction_dt_ = final_prediction_dt;
    return true;
}

FireCommand FireController::control(
    const predictor::BattlefieldSnapshot& snapshot,
    double current_time,
    const LatencyInfo& latency
)
{
    // 1. 更新云台状态
    double dt = (last_time_ > 0) ? (current_time - last_time_) : CONTROL_DT;
    gimbal_state_.update(snapshot.self_state.q_imu, dt);
    last_time_ = current_time;
    last_latency_ = latency;

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
        last_gate_debug_ = {};
        last_aim_ = {};
        last_armor_aim_ = {};
        last_plan_ = {};
        last_armor_id_ = -1;
        return no_target_command();
    }

    // 2. 计算自车速度（弹道解算需要）
    const Eigen::Vector3d self_velocity(
        snapshot.self_state.velocity.x(),
        snapshot.self_state.velocity.y(),
        0.0
    );

    // 3. 目标选择（同帧复用 target_id，避免 500Hz 下同帧反复切敌）
    const double img_age = std::max(0.0, current_time - snapshot.timestamp);
    const double prediction_dt_for_select =
        img_age + latency.send_to_control + latency.fire_to_hit;

    TargetSelection selection;
    if (has_cached_solution_ && snapshot.frame_id == last_solution_frame_id_
        && last_selection_.has_target && snapshot.is_valid(last_selection_.target_id))
    {
        selection = last_selection_;
    } else {
        selection = target_selector_.select(snapshot, gimbal_state_, prediction_dt_for_select);
    }
    last_selection_ = selection;

    // 4. 无目标处理
    if (!selection.has_target) {
        last_fail_stage_ = 1;  // 选目标失败
        has_cached_solution_ = false;
        last_armor_id_ = -1;
        if (snapshot.frame_id != last_no_target_frame_id_) {
            lost_count_++;
            last_no_target_frame_id_ = snapshot.frame_id;
        }
        if (lost_count_ > MAX_LOST_COUNT) {
            reset();
            last_fail_stage_ = 1;
        }
        last_gate_debug_ = {};
        last_aim_ = {};
        last_armor_aim_ = {};
        last_plan_ = {};
        return no_target_command();
    }
    lost_count_ = 0;
    last_no_target_frame_id_ = -1;

    if (!snapshot.is_valid(selection.target_id)) {
        last_fail_stage_ = 1;
        has_cached_solution_ = false;
        last_gate_debug_ = {};
        last_aim_ = {};
        last_armor_aim_ = {};
        last_plan_ = {};
        last_armor_id_ = -1;
        return no_target_command();
    }

    // 获取目标车辆的引用 (用索引访问，避免指针悬空问题)
    const auto& vehicle = snapshot.vehicles[selection.target_id];

    // 5. 选择 preferred armor（跨帧沿用绝对 id）
    int preferred_armor_idx = -1;
    if (last_armor_id_ >= 0) {
        for (int i = 0; i < vehicle.armor_count; ++i) {
            if (vehicle.armors[i].id == last_armor_id_) {
                preferred_armor_idx = i;
                break;
            }
        }
    }
    if (preferred_armor_idx < 0
        && vehicle.recommended_armor_idx >= 0
        && vehicle.recommended_armor_idx < vehicle.armor_count)
    {
        preferred_armor_idx = vehicle.recommended_armor_idx;
    }

    // 6. 延迟迭代 + 选板 + 弹道一体求解
    ArmorAimResult armor_result;
    AimResult aim;
    GimbalPlan plan;
    LatencyInfo solved_latency;
    if (!solve_aim_with_latency_iteration(
            snapshot,
            vehicle,
            self_velocity,
            current_time,
            latency,
            preferred_armor_idx,
            armor_result,
            aim,
            plan,
            solved_latency))
    {
        last_fail_stage_ = 2;  // 迭代求解失败（含选板/弹道失败）
        has_cached_solution_ = false;
        last_gate_debug_ = {};
        last_aim_ = {};
        last_armor_aim_ = {};
        last_plan_ = {};
        last_armor_id_ = -1;
        return no_target_command();
    }

    last_latency_ = solved_latency;
    last_plan_ = plan;
    last_aim_ = aim;
    last_armor_aim_ = armor_result;

    // 更新选择结果预测位置（用于调试显示）
    selection.predicted_pos = armor_result.target_pos;
    last_selection_ = selection;

    // 7. 开火判断
    last_solution_frame_id_ = snapshot.frame_id;
    last_target_confidence_ = vehicle.confidence;
    has_cached_solution_ = true;
    if (armor_result.armor_id >= 0) {
        last_armor_id_ = armor_result.armor_id;
    } else if (armor_result.armor_idx >= 0 && armor_result.armor_idx < vehicle.armor_count) {
        last_armor_id_ = vehicle.armors[armor_result.armor_idx].id;
    }
    bool can_fire = evaluate_fire_window(
        snapshot, solved_latency, vehicle, last_prediction_dt_, self_velocity
    );
    can_fire = can_fire && evaluate_rotate_back_gate(
        vehicle,
        last_prediction_dt_,
        solved_latency,
        snapshot.self_state.bullet_speed,
        self_velocity,
        snapshot.self_state.q_imu
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

    // 落点偏置 (配置单位: deg，运行时可热更新)
    const double aim_offset_yaw_deg = runtime_param::get_param<double>(
        "AutoAim.FireControl.AimOffset.yaw"
    );
    const double aim_offset_pitch_deg = runtime_param::get_param<double>(
        "AutoAim.FireControl.AimOffset.pitch"
    );
    const double aim_offset_yaw = aimer::math::deg2rad(aim_offset_yaw_deg);
    const double aim_offset_pitch = aimer::math::deg2rad(aim_offset_pitch_deg);

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
