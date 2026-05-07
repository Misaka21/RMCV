/**
 * @file fire_controller.cpp
 * @brief 火控主类实现 — 算法驱动，对齐 rm.cv.fans
 */

#include "fire_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

// ==================== 参数读取辅助 ====================

namespace {

double get_param_or(const std::string& name, double default_value) {
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (const auto* val = std::get_if<double>(&*ptr)) return *val;
    }
    return default_value;
}

}  // namespace

// ==================== 公共工具 ====================

double get_spin_window_rad(const predictor::TargetState& target) {
    const double top0_deg = (target.armor_count == 4)
        ? get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors4", 58.8888)
        : get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors_other", 75.0);
    const double top1_deg = get_param_or("AutoAim.FireControl.PID.top1_max_orientation_angle", 0.0);
    const double top2_deg = get_param_or("AutoAim.FireControl.PID.top2_max_orientation_angle", 0.0);

    switch (target.spin.level) {
        case predictor::SpinLevel::HIGH: return aimer::math::deg2rad(top2_deg);
        case predictor::SpinLevel::LOW:  return aimer::math::deg2rad(top1_deg);
        case predictor::SpinLevel::NONE:
        default:                          return aimer::math::deg2rad(top0_deg);
    }
}

// ==================== TargetCatcher ====================

void FireController::TargetCatcher::try_catch(int new_id, double current_time, double keep_time) {
    if (target_id < 0) {
        target_id = new_id;
        caught_time = current_time;
        return;
    }
    if (target_id == new_id) {
        caught_time = current_time;
        return;
    }
    // 切换目标: 旧目标需超过 keep_time 才释放
    if (current_time - caught_time > keep_time) {
        target_id = new_id;
        caught_time = current_time;
    }
}

int FireController::TargetCatcher::get(double current_time, double memorizing_time) const {
    if (target_id < 0) return -1;
    if (current_time - caught_time > memorizing_time) return -1;
    return target_id;
}

// ==================== 装甲板候选 ====================

std::vector<FireController::ArmorCandidate> FireController::get_candidates(
    const predictor::TargetState& target, double dt)
{
    std::vector<ArmorCandidate> candidates;
    candidates.reserve(target.armor_count);

    for (int i = 0; i < target.armor_count; ++i) {
        ArmorCandidate c;
        c.idx = i;
        c.id = target.armor_id(i);
        c.pos = target.predict_armor_position(i, dt);
        c.vel = target.predict_armor_velocity(i, dt);
        c.z_to_v = target.predicted_z_to_v(i, dt);
        c.width = target.armor_width(i);
        c.height = target.armor_height(i);
        c.visible = target.armor_visible(i);
        candidates.push_back(c);
    }
    return candidates;
}

// ==================== 瞄准配置 ====================

FireController::SpinAimProfile FireController::get_spin_profile(
    const predictor::TargetState& target) const
{
    SpinAimProfile p;

    // 窗口角 (对齐 rm.cv.fans: top0 大窗口, top1/top2 小或零窗口)
    const double top0_deg = (target.armor_count == 4)
        ? get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors4", 58.8888)
        : get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors_other", 75.0);
    const double top1_deg = get_param_or("AutoAim.FireControl.PID.top1_max_orientation_angle", 0.0);
    const double top2_deg = get_param_or("AutoAim.FireControl.PID.top2_max_orientation_angle", 0.0);

    const double top0_out = get_param_or("AutoAim.FireControl.PID.top0_max_out_error", 1.8);
    const double top1_out = get_param_or("AutoAim.FireControl.PID.top1_max_out_error", 0.6);
    const double top2_out = get_param_or("AutoAim.FireControl.PID.top2_max_out_error", 1.8);

    const double top0_swing = get_param_or("AutoAim.FireControl.PID.top0_max_swing_error", top0_out);
    const double top1_swing = get_param_or("AutoAim.FireControl.PID.top1_max_swing_error", top1_out);
    const double top2_swing = get_param_or("AutoAim.FireControl.PID.top2_max_swing_error", top2_out);

    const double top0_track = get_param_or("AutoAim.FireControl.PID.top0_max_tracking_error", 0.8);
    const double top1_track = get_param_or("AutoAim.FireControl.PID.top1_max_tracking_error", 0.8);
    const double top2_track = get_param_or("AutoAim.FireControl.PID.top2_max_tracking_error", 0.8);

    switch (target.spin.level) {
        case predictor::SpinLevel::HIGH:
            p.max_orientation_angle = aimer::math::deg2rad(top2_deg);
            p.max_out_error = std::max(0.0, top2_out);
            p.max_swing_error = std::max(0.0, top2_swing);
            p.max_tracking_error = std::max(0.0, top2_track);
            p.allow_indirect = true;
            break;
        case predictor::SpinLevel::LOW:
            p.max_orientation_angle = aimer::math::deg2rad(top1_deg);
            p.max_out_error = std::max(0.0, top1_out);
            p.max_swing_error = std::max(0.0, top1_swing);
            p.max_tracking_error = std::max(0.0, top1_track);
            p.allow_indirect = true;
            break;
        case predictor::SpinLevel::NONE:
        default:
            p.max_orientation_angle = aimer::math::deg2rad(top0_deg);
            p.max_out_error = std::max(0.0, top0_out);
            p.max_swing_error = std::max(0.0, top0_swing);
            p.max_tracking_error = std::max(0.0, top0_track);
            p.allow_indirect = false;
            break;
    }
    return p;
}

// ==================== 瞄准求解 ====================

ArmorAimResult FireController::solve_armor_aim(
    const predictor::TargetState& target,
    double predict_dt,
    const SpinAimProfile& profile,
    const Eigen::Quaterniond& q_imu,
    double bullet_speed,
    const Eigen::Vector3d& self_velocity,
    int preferred_idx)
{
    auto candidates = get_candidates(target, predict_dt);

    if (target.spin.active && std::abs(target.v_yaw) > 1e-4) {
        // 陀螺路径: direct → indirect
        ArmorAimResult direct = solve_direct(
            target, predict_dt, candidates,
            profile.max_orientation_angle,
            q_imu, bullet_speed, self_velocity, preferred_idx);
        if (direct.valid) return direct;

        if (profile.allow_indirect) {
            return solve_indirect(
                target, predict_dt, candidates,
                profile.max_orientation_angle, profile.max_out_error,
                q_imu, preferred_idx);
        }
        return ArmorAimResult{};
    }

    // 非陀螺路径
    return solve_non_spin(target, predict_dt, candidates, preferred_idx);
}

ArmorAimResult FireController::solve_direct(
    const predictor::TargetState& target,
    double predict_dt,
    const std::vector<ArmorCandidate>& candidates,
    double max_orientation_angle,
    const Eigen::Quaterniond& q_imu,
    double bullet_speed,
    const Eigen::Vector3d& self_velocity,
    int preferred_idx)
{
    ArmorAimResult result;
    result.mode = AimMode::DIRECT;

    // 1. 窗口过滤 (对齐 rm.cv.fans: |zn_to_armor| ≤ max_orientation_angle)
    const double window = std::max(0.0, max_orientation_angle);
    std::vector<int> in_window;
    for (const auto& c : candidates) {
        if (!c.pos.allFinite() || c.pos.squaredNorm() < 1e-9) continue;
        if (std::abs(c.z_to_v) <= window) {
            in_window.push_back(c.idx);
        }
    }
    if (in_window.empty()) return result;

    // 2. 评分: non-idle 优先, swing_cost 最小优先 (对齐 rm.cv.fans aim_cmp)
    struct Score { bool non_idle = false; double swing_cost = INFINITY; };
    int best_idx = -1;
    Score best_score;

    for (int idx : in_window) {
        const auto& c = candidates[idx];
        if (idx < 0 || idx >= static_cast<int>(candidates.size())) continue;

        double yaw_err = 0, pitch_err = 0;
        bool aim_valid = false;

        if (bullet_speed > 1e-3) {
            Eigen::Vector3d tv = aimer::tf::world_to_barrel_origin_world(c.pos, q_imu);
            AimResult aim = ::fire_control::trajectory::solve(tv, bullet_speed, self_velocity);
            if (aim.valid) {
                yaw_err = GimbalState::normalize_angle(aim.yaw - gimbal_state_.yaw);
                pitch_err = aim.pitch - gimbal_state_.pitch;
                aim_valid = true;
            }
        }
        if (!aim_valid) {
            yaw_err = GimbalState::normalize_angle(
                std::atan2(c.pos.y(), c.pos.x()) - gimbal_state_.yaw);
            pitch_err = std::atan2(c.pos.z(), std::hypot(c.pos.x(), c.pos.y()))
                - gimbal_state_.pitch;
        }

        Score s;
        s.non_idle = std::abs(yaw_err) < M_PI_2 && std::abs(pitch_err) < M_PI_2;
        s.swing_cost = std::hypot(yaw_err, pitch_err);

        bool is_better = false;
        if (s.non_idle != best_score.non_idle) {
            is_better = s.non_idle;
        } else {
            is_better = s.swing_cost < best_score.swing_cost;
        }

        if (best_idx < 0 || is_better) {
            best_score = s;
            best_idx = idx;
        }
    }

    if (best_idx < 0) return result;

    // 3. 迟滞保持 (对齐 rm.cv.fans: 偏好板不差于最优+迟滞则保持)
    const double hysteresis = aimer::math::deg2rad(
        std::max(0.0, get_param_or("AutoAim.FireControl.PID.direct_switch_hysteresis_deg", 2.0)));
    if (preferred_idx >= 0 && preferred_idx != best_idx) {
        // 检查 preferred 是否也在窗口内且 non-idle
        auto it = std::find(in_window.begin(), in_window.end(), preferred_idx);
        if (it != in_window.end()) {
            const auto& pc = candidates[preferred_idx];
            double py = 0, pp = 0;
            bool pv = false;
            if (bullet_speed > 1e-3) {
                Eigen::Vector3d tv = aimer::tf::world_to_barrel_origin_world(pc.pos, q_imu);
                AimResult pa = ::fire_control::trajectory::solve(tv, bullet_speed, self_velocity);
                if (pa.valid) {
                    py = GimbalState::normalize_angle(pa.yaw - gimbal_state_.yaw);
                    pp = pa.pitch - gimbal_state_.pitch;
                    pv = true;
                }
            }
            if (!pv) {
                py = GimbalState::normalize_angle(
                    std::atan2(pc.pos.y(), pc.pos.x()) - gimbal_state_.yaw);
                pp = std::atan2(pc.pos.z(), std::hypot(pc.pos.x(), pc.pos.y()))
                    - gimbal_state_.pitch;
            }
            bool p_non_idle = std::abs(py) < M_PI_2 && std::abs(pp) < M_PI_2;
            double p_cost = std::hypot(py, pp);
            if (p_non_idle && (!best_score.non_idle || p_cost <= best_score.swing_cost + hysteresis)) {
                best_idx = preferred_idx;
            }
        }
    }

    const auto& c = candidates[best_idx];
    result.valid = true;
    result.armor_idx = best_idx;
    result.armor_id = c.id;
    result.target_pos = c.pos;
    result.target_vel = c.vel;
    result.z_to_v = c.z_to_v;
    result.armor_width = c.width;
    result.armor_height = c.height;
    result.time_to_fire = 0;
    return result;
}

ArmorAimResult FireController::solve_indirect(
    const predictor::TargetState& target,
    double predict_dt,
    const std::vector<ArmorCandidate>& candidates,
    double max_orientation_angle,
    double max_out_error,
    const Eigen::Quaterniond& q_imu,
    int preferred_idx)
{
    ArmorAimResult result;
    result.mode = AimMode::INDIRECT;

    const double omega = target.v_yaw;
    if (std::abs(omega) < 1e-4) return result;

    // 对齐 rm.cv.fans: 窗口边界方向
    const double zn_to_wait = (omega > 0.0) ? -max_orientation_angle : +max_orientation_angle;

    const double sample_width = (candidates.empty())
        ? 0.136 : candidates[0].width;
    const double switch_hyst = aimer::math::deg2rad(
        std::max(0.0, get_param_or("AutoAim.FireControl.PID.indirect_switch_hysteresis_deg", 5.0)));

    // 找最早进入窗口的板 (armor_to_wait 最小)
    int best_idx = -1;
    double best_to_wait = std::numeric_limits<double>::infinity();
    double preferred_to_wait = std::numeric_limits<double>::infinity();

    for (const auto& c : candidates) {
        // 计算半径
        const Eigen::Vector3d center = target.predict_center(predict_dt);
        const Eigen::Vector3d offset = c.pos - center;
        const double radius = std::hypot(offset.x(), offset.y());
        if (radius < 1e-5) continue;

        // 对齐 rm.cv.fans: max_out_angle = sample_width/2 * max_out_error / radius
        const double out_angle = sample_width * 0.5 * max_out_error / radius;

        // armor_to_wait ∈ (-out_angle, 2π - out_angle)
        const double armor_to_wait =
            aimer::math::reduced_angle((omega > 0.0 ? zn_to_wait - c.z_to_v : c.z_to_v - zn_to_wait)
                        - M_PI + out_angle) + M_PI - out_angle;

        if (armor_to_wait < best_to_wait) {
            best_to_wait = armor_to_wait;
            best_idx = c.idx;
        }

        if (c.idx == preferred_idx) {
            preferred_to_wait = armor_to_wait;
        }
    }

    if (best_idx < 0 || !std::isfinite(best_to_wait)) return result;

    // 迟滞保持
    if (preferred_idx >= 0 && std::isfinite(preferred_to_wait)) {
        if (preferred_to_wait <= best_to_wait + switch_hyst) {
            best_idx = preferred_idx;
            best_to_wait = preferred_to_wait;
        }
    }

    // emerge 位置: 推进装甲板到入窗时刻 (对齐 rm.cv.fans LmtdTopModel)
    const double time_to_emerge = best_to_wait / std::abs(omega);
    const double emerge_dt = predict_dt + time_to_emerge;
    const Eigen::Vector3d emerge_pos = target.predict_armor_position(best_idx, emerge_dt);

    if (!emerge_pos.allFinite() || emerge_pos.squaredNorm() < 1e-9) {
        return ArmorAimResult{};
    }

    const auto& c = candidates[best_idx];
    result.valid = true;
    result.armor_idx = best_idx;
    result.armor_id = c.id;
    result.target_pos = emerge_pos;
    result.target_vel = target.velocity;  // indirect: 只跟车心速度
    result.z_to_v = c.z_to_v;
    result.time_to_fire = time_to_emerge;
    result.armor_width = c.width;
    result.armor_height = c.height;
    return result;
}

ArmorAimResult FireController::solve_non_spin(
    const predictor::TargetState& target,
    double predict_dt,
    const std::vector<ArmorCandidate>& candidates,
    int preferred_idx)
{
    ArmorAimResult result;
    result.mode = AimMode::DIRECT;

    // 可见板中选最正对的 (|z_to_v| 最小)
    int best_idx = -1;
    double best_z = std::numeric_limits<double>::infinity();

    for (const auto& c : candidates) {
        if (!c.visible) continue;
        if (std::abs(c.z_to_v) < best_z) {
            best_z = std::abs(c.z_to_v);
            best_idx = c.idx;
        }
    }

    if (best_idx < 0) return result;

    const auto& c = candidates[best_idx];
    result.valid = true;
    result.armor_idx = best_idx;
    result.armor_id = c.id;
    result.target_pos = c.pos;
    result.target_vel = c.vel;
    result.z_to_v = c.z_to_v;
    result.armor_width = c.width;
    result.armor_height = c.height;
    result.time_to_fire = 0;
    return result;
}

double FireController::compute_swing_cost(
    const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& q_imu,
    double bullet_speed,
    const Eigen::Vector3d& self_velocity) const
{
    if (bullet_speed > 1e-3) {
        Eigen::Vector3d tv = aimer::tf::world_to_barrel_origin_world(target_pos, q_imu);
        AimResult aim = ::fire_control::trajectory::solve(tv, bullet_speed, self_velocity);
        if (aim.valid) {
            double dy = GimbalState::normalize_angle(aim.yaw - gimbal_state_.yaw);
            double dp = aim.pitch - gimbal_state_.pitch;
            return std::hypot(dy, dp);
        }
    }
    double dy = GimbalState::normalize_angle(
        std::atan2(target_pos.y(), target_pos.x()) - gimbal_state_.yaw);
    double dp = std::atan2(target_pos.z(), std::hypot(target_pos.x(), target_pos.y()))
        - gimbal_state_.pitch;
    return std::hypot(dy, dp);
}

// ==================== 弹道解算 ====================

AimResult FireController::solve_trajectory(
    const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& q_imu,
    double bullet_speed,
    const Eigen::Vector3d& self_velocity) const
{
    Eigen::Vector3d tv = aimer::tf::world_to_barrel_origin_world(target_pos, q_imu);
    return ::fire_control::trajectory::solve(tv, bullet_speed, self_velocity);
}

// ==================== 开火门控 ====================

bool FireController::evaluate_fire_gate(
    const predictor::BattlefieldSnapshot& snapshot,
    const predictor::TargetState& target,
    const LatencyInfo& latency,
    const SpinAimProfile& profile,
    double prediction_dt,
    const Eigen::Vector3d& self_velocity)
{
    last_gate_debug_ = {};

    if (!last_aim_.valid || !last_armor_aim_.valid) return false;

    const auto& gs = gimbal_state_;
    const auto& aim = last_aim_;
    const auto& armor = last_armor_aim_;
    const double bullet_speed = snapshot.self_state.bullet_speed;
    const auto& q_imu = snapshot.self_state.q_imu;

    // ---- 1. 基础跟踪门控 ----
    const double aim_off_yaw = aimer::math::deg2rad(runtime_param::get_param<double>(
        "AutoAim.FireControl.AimOffset.yaw"));
    const double aim_off_pitch = aimer::math::deg2rad(runtime_param::get_param<double>(
        "AutoAim.FireControl.AimOffset.pitch"));

    double yaw_err = GimbalState::normalize_angle((aim.yaw + aim_off_yaw) - gs.yaw);
    double pitch_err = (aim.pitch + aim_off_pitch) - gs.pitch;

    last_gate_debug_.tracking.confidence = target.confidence;
    last_gate_debug_.tracking.min_confidence = runtime_param::get_param<double>(
        "AutoAim.FireControl.min_confidence");
    last_gate_debug_.tracking.conf_ok = true;

    if (std::abs(yaw_err) >= M_PI_2 || std::abs(pitch_err) >= M_PI_2) {
        last_gate_debug_.tracking.angle_ok = false;
        return false;
    }
    last_gate_debug_.tracking.angle_ok = true;

    const double dist = std::max(1e-3, aim.distance);
    last_gate_debug_.tracking.hit_offset_yaw = dist * std::abs(std::tan(yaw_err));
    last_gate_debug_.tracking.hit_offset_pitch = dist * std::abs(std::tan(pitch_err));

    const double error_rate = runtime_param::get_param<double>(
        "AutoAim.FireControl.error_rate");
    last_gate_debug_.tracking.error_rate = error_rate;

    const double cos_inclined = std::abs(std::cos(armor.z_to_v));
    last_gate_debug_.tracking.yaw_limit = (armor.armor_width * 0.5) * cos_inclined * error_rate;
    last_gate_debug_.tracking.pitch_limit = (armor.armor_height * 0.5) * error_rate;
    last_gate_debug_.tracking.yaw_ok =
        last_gate_debug_.tracking.hit_offset_yaw < last_gate_debug_.tracking.yaw_limit;
    last_gate_debug_.tracking.pitch_ok =
        last_gate_debug_.tracking.hit_offset_pitch < last_gate_debug_.tracking.pitch_limit;

    bool can_fire = last_gate_debug_.pass();
    last_gate_debug_.allow_fire_ok = snapshot.self_state.allow_fire;
    can_fire = can_fire && last_gate_debug_.allow_fire_ok;

    // ---- 2. 陀螺 swing/out 门控 (对齐 rm.cv.fans) ----
    if (target.spin.active && std::abs(target.v_yaw) > 1e-4) {
        const double control_to_fire = std::max(0.0, latency.control_to_fire);
        const double hit_dt = prediction_dt + control_to_fire;

        last_gate_debug_.swing_error_rate = profile.max_swing_error;
        last_gate_debug_.out_error_rate = profile.max_out_error;

        // 命中时刻装甲板位置
        const Eigen::Vector3d hit_pos = target.predict_armor_position(
            armor.armor_idx, hit_dt);
        if (!hit_pos.allFinite()) { can_fire = false; }
        else {
            // swing: 命中点与当前枪口的偏差
            AimResult hit_aim = solve_trajectory(
                hit_pos, q_imu, bullet_speed, self_velocity);
            if (!hit_aim.valid) { can_fire = false; }
            else {
                double sy = GimbalState::normalize_angle(hit_aim.yaw - gs.yaw);
                double sp = hit_aim.pitch - gs.pitch;
                if (std::abs(sy) >= M_PI_2 || std::abs(sp) >= M_PI_2) {
                    last_gate_debug_.swing_ok = false;
                    can_fire = false;
                } else {
                    const double sd = std::max(1e-3, hit_aim.distance);
                    last_gate_debug_.swing_offset_yaw = sd * std::abs(std::tan(sy));
                    last_gate_debug_.swing_offset_pitch = sd * std::abs(std::tan(sp));
                    const double sc = std::abs(std::cos(armor.z_to_v));
                    last_gate_debug_.swing_yaw_limit =
                        (armor.armor_width * 0.5) * sc * profile.max_swing_error;
                    last_gate_debug_.swing_pitch_limit =
                        (armor.armor_height * 0.5) * profile.max_swing_error;
                    last_gate_debug_.swing_ok =
                        last_gate_debug_.swing_offset_yaw < last_gate_debug_.swing_yaw_limit
                        && last_gate_debug_.swing_offset_pitch < last_gate_debug_.swing_pitch_limit;
                    if (!last_gate_debug_.swing_ok) can_fire = false;
                }
            }

            // out: emerging 瞄点与装甲板中心的偏差 (对齐 rm.cv.fans)
            if (can_fire) {
                const Eigen::Vector3d hit_center = target.predict_armor_position(
                    armor.armor_idx, hit_dt);
                AimResult center_aim = solve_trajectory(
                    hit_center, q_imu, bullet_speed, self_velocity);
                if (center_aim.valid) {
                    double dy = GimbalState::normalize_angle(hit_aim.yaw - center_aim.yaw);
                    double dp = hit_aim.pitch - center_aim.pitch;
                    if (std::abs(dy) < M_PI_2 && std::abs(dp) < M_PI_2) {
                        const double cd = std::max(1e-3, center_aim.distance);
                        last_gate_debug_.out_offset_yaw = cd * std::abs(std::tan(dy));
                        last_gate_debug_.out_offset_pitch = cd * std::abs(std::tan(dp));
                        const double oc = std::abs(std::cos(armor.z_to_v));
                        last_gate_debug_.out_yaw_limit =
                            (armor.armor_width * 0.5) * oc * profile.max_out_error;
                        last_gate_debug_.out_pitch_limit =
                            (armor.armor_height * 0.5) * profile.max_out_error;
                        last_gate_debug_.out_ok =
                            last_gate_debug_.out_offset_yaw < last_gate_debug_.out_yaw_limit
                            && last_gate_debug_.out_offset_pitch < last_gate_debug_.out_pitch_limit;
                        if (!last_gate_debug_.out_ok) can_fire = false;
                    }
                }
            }
        }
    } else {
        last_gate_debug_.swing_ok = true;
        last_gate_debug_.out_ok = true;
    }

    return can_fire;
}

// ==================== 回转禁发门控 ====================

bool FireController::evaluate_rotate_back_gate(
    const predictor::TargetState& target,
    double prediction_dt,
    const LatencyInfo& latency,
    double bullet_speed,
    const Eigen::Vector3d& self_velocity,
    const Eigen::Quaterniond& q_imu)
{
    last_rotate_back_ok_ = true;
    last_rotate_back_active_ = false;
    last_rotate_back_start_ = 0;
    last_rotate_back_end_ = 0;
    last_rotate_back_command_time_ = prediction_dt + std::max(0.0, latency.control_to_fire);

    if (!last_aim_.valid || !last_armor_aim_.valid) return true;
    if (!target.spin.active || target.spin.level == predictor::SpinLevel::NONE) return true;
    if (std::abs(target.v_yaw) < 1e-4 || bullet_speed <= 1e-3) return true;

    const double control_to_fire = std::max(0.0, latency.control_to_fire);
    if (control_to_fire <= 1e-6) return true;

    // 对齐 rm.cv.fans: 比较 water_gun_hit 和 command_hit 两个时刻的装甲板角位移
    const double t_water = prediction_dt;
    const double t_cmd = prediction_dt + control_to_fire;
    last_rotate_back_command_time_ = t_cmd;

    const double omega = target.v_yaw;
    const double armor_yaw_water = target.armor_yaw(last_armor_aim_.armor_idx, t_water);
    const double armor_yaw_cmd = target.armor_yaw(last_armor_aim_.armor_idx, t_cmd);
    const double armor_rotate = aimer::math::reduced_angle(armor_yaw_cmd - armor_yaw_water);

    // 角速度方向与角位移方向相同 → 不在回转
    if (std::signbit(omega) == std::signbit(armor_rotate)) return true;

    // 回转: 计算回转时间窗口
    const double max_orientation_angle = get_spin_window_rad(target);
    const double zn_to_armor_water = last_armor_aim_.z_to_v;
    const double zn_to_rotate_back = (omega > 0.0)
        ? +max_orientation_angle : -max_orientation_angle;
    const double armor_water_to_rotate_back = aimer::math::reduced_angle(
        zn_to_rotate_back - zn_to_armor_water);

    const double time_start = t_water + armor_water_to_rotate_back / omega;
    if (!std::isfinite(time_start) || time_start >= t_cmd) return true;

    // 计算回转耗时
    const Eigen::Vector3d pos_start = target.predict_armor_position(
        last_armor_aim_.armor_idx, time_start);
    AimResult aim_start = solve_trajectory(pos_start, q_imu, bullet_speed, self_velocity);
    if (!aim_start.valid) return true;

    const Eigen::Vector3d cmd_pos = target.predict_armor_position(
        last_armor_aim_.armor_idx, t_cmd);
    AimResult aim_cmd = solve_trajectory(cmd_pos, q_imu, bullet_speed, self_velocity);
    if (!aim_cmd.valid) return true;

    const double yaw_rotate = GimbalState::normalize_angle(aim_cmd.yaw - aim_start.yaw);
    const double rotate_a = get_param_or("AutoAim.FireControl.PID.angle_to_rotate_time_a", 1.79e-3);
    const double rotate_b = get_param_or("AutoAim.FireControl.PID.angle_to_rotate_time_b", 0.093);
    const double rotate_time = rotate_a * std::abs(yaw_rotate) * 180.0 / M_PI + rotate_b;

    const double time_end = time_start + std::max(0.0, rotate_time);
    last_rotate_back_active_ = true;
    last_rotate_back_start_ = time_start;
    last_rotate_back_end_ = time_end;

    if (time_start < t_cmd && t_cmd < time_end) {
        last_rotate_back_ok_ = false;
    }
    return last_rotate_back_ok_;
}

// ==================== 无目标指令 ====================

FireCommand FireController::no_target_command() {
    FireCommand cmd;
    cmd.control_enabled = false;
    cmd.allow_fire = false;
    cmd.fire_now = false;
    cmd.target_id = -1;
    return cmd;
}

// ==================== 主入口 ====================

FireCommand FireController::control(
    const predictor::BattlefieldSnapshot& snapshot,
    double current_time,
    const LatencyInfo& latency)
{
    // 1. 更新云台
    double dt = (last_time_ > 0) ? (current_time - last_time_) : CONTROL_DT;
    gimbal_state_.update(snapshot.self_state.q_imu, dt);
    last_time_ = current_time;
    last_latency_ = latency;

    const double bullet_speed = snapshot.self_state.bullet_speed;
    const Eigen::Vector3d self_velocity(
        snapshot.self_state.velocity.x(),
        snapshot.self_state.velocity.y(), 0.0);

    // 2. 选敌: 对齐 rm.cv.fans TargetCatcher
    //    当前帧最中心目标
    int center_id = -1;
    double center_best = std::numeric_limits<double>::infinity();
    snapshot.for_each_valid([&](int id, const predictor::TargetState& t) {
        if (!snapshot.is_detected(id)) return;
        // 选离图像中心最近的 (t.position 相对相机)
        const Eigen::Vector3d pos = t.predict_center(0);
        if (!pos.allFinite()) return;
        // 用 gimbal 指向作为"图像中心"
        double dy = GimbalState::normalize_angle(
            std::atan2(pos.y(), pos.x()) - gimbal_state_.yaw);
        double dp = std::atan2(pos.z(), std::hypot(pos.x(), pos.y()))
            - gimbal_state_.pitch;
        double cost = std::hypot(dy, dp);
        if (cost < center_best) { center_best = cost; center_id = id; }
    });

    const double keep_time = get_param_or(
        "AutoAim.FireControl.TargetSelector.keep_time", 0.1);
    const double memorizing_time = get_param_or(
        "AutoAim.FireControl.TargetSelector.memorizing_time", 5.0);

    if (center_id >= 0) {
        catcher_.try_catch(center_id, current_time, keep_time);
    }
    int target_id = catcher_.get(current_time, memorizing_time);

    if (target_id < 0 || !snapshot.is_valid(target_id)) {
        last_fail_stage_ = 1;
        last_selection_ = {};
        last_aim_ = {};
        last_armor_aim_ = {};
        last_plan_ = {};
        last_gate_debug_ = {};
        last_armor_id_ = -1;
        return no_target_command();
    }

    const auto* target = snapshot.find_target(target_id);
    if (target == nullptr) {
        last_fail_stage_ = 1;
        last_selection_ = {};
        last_aim_ = {};
        last_armor_aim_ = {};
        last_plan_ = {};
        last_gate_debug_ = {};
        last_armor_id_ = -1;
        return no_target_command();
    }

    const auto& vehicle = *target;

    // 3. 延迟迭代 (弹道飞行时间收敛)
    LatencyInfo iter_latency = latency;
    const double img_age = std::max(0.0, current_time - snapshot.timestamp);
    const int iter_count = static_cast<int>(std::clamp(
        get_param_or("AutoAim.FireControl.Latency.iterations", 2.0), 1.0, 5.0));

    // 跨帧保持 armor_id 偏好
    int preferred_idx = -1;
    if (last_armor_id_ >= 0) {
        for (int i = 0; i < vehicle.armor_count; ++i) {
            if (vehicle.armor_id(i) == last_armor_id_) {
                preferred_idx = i; break;
            }
        }
    }

    SpinAimProfile profile = get_spin_profile(vehicle);
    ArmorAimResult armor;
    AimResult aim;
    bool spin_iter_locked = false;
    int iter_preferred = preferred_idx;

    for (int iter = 0; iter < iter_count; ++iter) {
        const double predict_dt = img_age + iter_latency.send_to_control
            + iter_latency.fire_to_hit;

        armor = solve_armor_aim(
            vehicle, predict_dt, profile,
            snapshot.self_state.q_imu,
            bullet_speed, self_velocity,
            iter_preferred);

        if (!armor.valid) {
            last_fail_stage_ = 2;
            last_selection_ = {true, target_id, 1.0, {}};
            last_aim_ = {};
            last_armor_aim_ = {};
            last_plan_ = {};
            last_armor_id_ = -1;
            return no_target_command();
        }

        aim = solve_trajectory(
            armor.target_pos, snapshot.self_state.q_imu,
            bullet_speed, self_velocity);

        if (!aim.valid) {
            last_fail_stage_ = 3;
            last_selection_ = {true, target_id, 1.0, armor.target_pos};
            last_aim_ = {};
            last_armor_aim_ = armor;
            last_plan_ = {};
            last_armor_id_ = armor.armor_id;
            return no_target_command();
        }

        if (std::isfinite(aim.fly_time) && aim.fly_time > 0.0) {
            iter_latency.set_fly_time(aim.fly_time);
        }

        // 陀螺: 锁存第一次迭代的选板
        if (vehicle.spin.active && !spin_iter_locked) {
            iter_preferred = armor.armor_idx;
            spin_iter_locked = true;
        } else if (!vehicle.spin.active) {
            iter_preferred = armor.armor_idx;
        }
    }

    // 最终求解
    const double final_predict_dt = img_age + iter_latency.send_to_control
        + iter_latency.fire_to_hit;
    armor = solve_armor_aim(
        vehicle, final_predict_dt, profile,
        snapshot.self_state.q_imu,
        bullet_speed, self_velocity,
        iter_preferred);

    if (!armor.valid) {
        last_fail_stage_ = 2;
        last_aim_ = {};
        last_armor_aim_ = {};
        last_plan_ = {};
        return no_target_command();
    }

    aim = solve_trajectory(
        armor.target_pos, snapshot.self_state.q_imu,
        bullet_speed, self_velocity);

    if (!aim.valid) {
        last_fail_stage_ = 3;
        last_aim_ = {};
        last_armor_aim_ = armor;
        last_plan_ = {};
        return no_target_command();
    }

    if (std::isfinite(aim.fly_time) && aim.fly_time > 0.0) {
        iter_latency.set_fly_time(aim.fly_time);
    }
    last_latency_ = iter_latency;
    last_prediction_dt_ = final_predict_dt;

    // 4. MPC 规划 (对齐 sp_vision_25 Planner)
    const double hit_offset = iter_latency.send_to_control + iter_latency.fire_to_hit;
    planner_.build_reference(
        vehicle, armor.armor_idx, img_age, hit_offset,
        bullet_speed, self_velocity, snapshot.self_state.q_imu);
    PlannerOutput planner_out = planner_.step(gimbal_state_);
    last_planner_output_ = planner_out;

    // 5. 构建 GimbalPlan (从 MPC 输出)
    GimbalPlan plan;
    if (planner_out.valid) {
        plan.valid = true;
        plan.yaw = planner_out.yaw;
        plan.pitch = planner_out.pitch;
        plan.yaw_vel = planner_out.yaw_vel;
        plan.pitch_vel = planner_out.pitch_vel;
        plan.yaw_acc = planner_out.yaw_acc;
        plan.pitch_acc = planner_out.pitch_acc;
    } else {
        // MPC 未就绪: 退化为原始瞄准点 + 零速度
        plan.valid = true;
        plan.yaw = aim.yaw;
        plan.pitch = aim.pitch;
        plan.yaw_vel = 0;
        plan.pitch_vel = 0;
    }

    // 6. 缓存调试状态
    last_selection_ = {true, target_id, 1.0, armor.target_pos};
    last_aim_ = aim;
    last_armor_aim_ = armor;
    last_plan_ = plan;
    last_armor_id_ = armor.armor_id;

    // 7. 开火判断
    bool can_fire = evaluate_fire_gate(
        snapshot, vehicle, iter_latency, profile,
        final_predict_dt, self_velocity);

    can_fire = can_fire && evaluate_rotate_back_gate(
        vehicle, final_predict_dt, iter_latency,
        bullet_speed, self_velocity, snapshot.self_state.q_imu);

    last_fail_stage_ = 9;

    // 8. 生成指令
    FireCommand cmd;
    cmd.control_enabled = true;

    const double off_yaw = aimer::math::deg2rad(runtime_param::get_param<double>(
        "AutoAim.FireControl.AimOffset.yaw"));
    const double off_pitch = aimer::math::deg2rad(runtime_param::get_param<double>(
        "AutoAim.FireControl.AimOffset.pitch"));

    // MPC 已包含完整轨迹 (位置+速度+加速度), 不再需要 additional_predict_time 前馈
    // aim_offset 在校准偏差时为非零, 加在 MPC 输出之上
    cmd.yaw = static_cast<float>(plan.yaw + off_yaw);
    cmd.yaw_vel = static_cast<float>(plan.yaw_vel);
    cmd.yaw_acc = static_cast<float>(plan.yaw_acc);
    cmd.pitch = static_cast<float>(plan.pitch + off_pitch);
    cmd.pitch_vel = static_cast<float>(plan.pitch_vel);
    cmd.pitch_acc = static_cast<float>(plan.pitch_acc);

    cmd.allow_fire = true;
    cmd.fire_now = can_fire;
    cmd.target_id = target_id;
    cmd.tracking_error = static_cast<float>(
        last_gate_debug_.pass() ? 0.0
        : std::hypot(last_gate_debug_.tracking.hit_offset_yaw,
                     last_gate_debug_.tracking.hit_offset_pitch));
    cmd.confidence = static_cast<float>(vehicle.confidence);

    return cmd;
}

// ==================== 重置 ====================

void FireController::reset() {
    catcher_.reset();
    gimbal_state_ = {};
    planner_.reset();
    last_time_ = 0;
    last_selection_ = {};
    last_aim_ = {};
    last_armor_aim_ = {};
    last_plan_ = {};
    last_gate_debug_ = {};
    last_latency_ = {};
    last_prediction_dt_ = 0;
    last_armor_id_ = -1;
    last_fail_stage_ = 0;
    last_rotate_back_ok_ = true;
    last_rotate_back_active_ = false;
    last_rotate_back_start_ = 0;
    last_rotate_back_end_ = 0;
    last_rotate_back_command_time_ = 0;
}

}  // namespace autoaim::fire_control
