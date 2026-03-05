/**
 * @file armor_aim.cpp
 * @brief 装甲板瞄准逻辑实现
 */

#include "armor_aim.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include <fmt/format.h>

#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/debug/logger.hpp"

namespace autoaim::fire_control {

namespace {

struct TopAimProfile {
    double max_orientation_angle = 0.0;  // rad
    double max_out_error = 0.0;
    bool allow_indirect = false;
    bool direct_visible_only = true;
};

double get_param_or(const std::string& name, double default_value)
{
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (auto* val = std::get_if<double>(&*ptr)) {
            return *val;
        }
    }
    return default_value;
}

bool get_param_or(const std::string& name, bool default_value)
{
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (auto* val = std::get_if<bool>(&*ptr)) {
            return *val;
        }
    }
    return default_value;
}

double now_sec()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

double center_cost(
    const Eigen::Vector3d& pos,
    const ::fire_control::GimbalState& gimbal
)
{
    const double yaw = std::atan2(pos.y(), pos.x());
    const double pitch = std::atan2(pos.z(), std::hypot(pos.x(), pos.y()));
    const double dyaw = ::fire_control::GimbalState::normalize_angle(yaw - gimbal.yaw);
    const double dpitch = pitch - gimbal.pitch;
    return std::hypot(dyaw, dpitch);
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
    if (!vehicle.spin.active || std::abs(vehicle.spin.omega) < 0.1) {
        return armor.z_to_v;
    }

    const Eigen::Vector3d pos = vehicle.predict_armor_position(armor_idx, predict_dt);
    const double armor_yaw = armor.yaw + vehicle.spin.omega * predict_dt;
    const double view_yaw = std::atan2(pos.y(), pos.x());
    return aimer::math::reduced_angle(armor_yaw - view_yaw - M_PI);
}

TopAimProfile get_top_aim_profile(const predictor::VehicleState& vehicle) {
    const double top0_deg = (vehicle.armor_count == 4)
        ? get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors4", 58.8888)
        : get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors_other", 75.0);
    const double top0_out = get_param_or("AutoAim.FireControl.PID.top0_max_out_error", 1.8);

    const double top1_deg = get_param_or("AutoAim.FireControl.PID.top1_max_orientation_angle", 0.0);
    const double top1_out = get_param_or("AutoAim.FireControl.PID.top1_max_out_error", 0.6);

    const double top2_deg = get_param_or("AutoAim.FireControl.PID.top2_max_orientation_angle", 0.0);
    const double top2_out = get_param_or("AutoAim.FireControl.PID.top2_max_out_error", 1.8);

    TopAimProfile profile;
    switch (vehicle.spin.level) {
        case predictor::SpinLevel::HIGH:
            profile.max_orientation_angle = aimer::math::deg2rad(top2_deg);
            profile.max_out_error = std::max(0.0, top2_out);
            profile.allow_indirect = true;
            // 对齐 rm.cv.fans: spin 场景 direct 在预测状态空间选板，不强依赖可见性。
            profile.direct_visible_only = false;
            break;
        case predictor::SpinLevel::LOW:
            profile.max_orientation_angle = aimer::math::deg2rad(top1_deg);
            profile.max_out_error = std::max(0.0, top1_out);
            // 对齐 rm.cv.fans (top1): direct 失败后允许 indirect 等待进入窗口。
            profile.allow_indirect = true;
            profile.direct_visible_only = false;
            break;
        case predictor::SpinLevel::NONE:
        default:
            profile.max_orientation_angle = aimer::math::deg2rad(top0_deg);
            profile.max_out_error = std::max(0.0, top0_out);
            profile.allow_indirect = false;
            profile.direct_visible_only = true;
            break;
    }
    return profile;
}

}  // namespace

ArmorAimResult ArmorAim::compute(
    const predictor::VehicleState& vehicle,
    double predict_dt
) const
{
    return compute(vehicle, predict_dt, nullptr, nullptr, -1);
}

ArmorAimResult ArmorAim::compute(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    const Eigen::Quaterniond* q_imu,
    int preferred_armor_idx
) const
{
    if (!vehicle.valid) {
        return ArmorAimResult{};
    }

    // 是否陀螺仅信 predictor 输出，避免火控重复阈值判定
    if (!vehicle.spin.active) {
        return compute_non_spin(vehicle, predict_dt, gimbal, q_imu, preferred_armor_idx);
    }
    return compute_spin(vehicle, predict_dt, gimbal, q_imu, preferred_armor_idx);
}

ArmorAimResult ArmorAim::compute(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int preferred_armor_idx
) const
{
    return compute(vehicle, predict_dt, gimbal, nullptr, preferred_armor_idx);
}

ArmorAimResult ArmorAim::compute_non_spin(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    const Eigen::Quaterniond* q_imu,
    int /* preferred_armor_idx */
) const
{
    (void)q_imu;
    // 非陀螺: 直接喵中心（仅可见）
    return compute_direct(
        vehicle, predict_dt, gimbal, -1,
        /*use_orientation_window=*/false,
        /*max_orientation_angle=*/0.0,
        /*visible_only=*/true,
        /*strict_orientation_window=*/false
    );
}

ArmorAimResult ArmorAim::compute_spin(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    const Eigen::Quaterniond* q_imu,
    int preferred_armor_idx
) const
{
    // 策略（对齐 rm.cv.fans）：
    // top0: direct-only
    // top1/top2: direct -> indirect
    const TopAimProfile profile = get_top_aim_profile(vehicle);

    // 对齐 rm.cv.fans：
    // 无论窗口角是否为 0，先尝试 direct(窗口内)，失败再回退 indirect（top1/top2）。
    // 当 max_orientation_angle=0 时，direct 仅在板恰好进入中心线时命中；否则走 indirect 等待。
    ArmorAimResult direct = compute_direct(
        vehicle, predict_dt, gimbal, preferred_armor_idx,
        /*use_orientation_window=*/true,
        profile.max_orientation_angle,
        /*visible_only=*/profile.direct_visible_only,
        /*strict_orientation_window=*/true
    );
    if (direct.valid) {
        return direct;
    }

    // top1/top2: direct 失败后用 indirect（等待板进入窗口）
    if (profile.allow_indirect) {
        ArmorAimResult indirect = compute_indirect(
            vehicle,
            predict_dt,
            gimbal,
            q_imu,
            profile.max_orientation_angle,
            profile.max_out_error,
            preferred_armor_idx
        );
        if (indirect.valid) {
            return indirect;
        }
    }

    // 与 rm.cv.fans 对齐：无 direct/indirect 可用时返回无效（上层进入 HOLD）
    return ArmorAimResult{};
}

ArmorAimResult ArmorAim::compute_direct(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int preferred_armor_idx,
    bool use_orientation_window,
    double max_orientation_angle,
    bool visible_only,
    bool strict_orientation_window
) const
{
    ArmorAimResult result;
    result.mode = AimMode::DIRECT;

    std::vector<int> candidate_indices;
    candidate_indices.reserve(vehicle.armor_count);
    for (int i = 0; i < vehicle.armor_count; ++i) {
        if (!visible_only || vehicle.armors[i].visible) {
            candidate_indices.push_back(i);
        }
    }
    if (candidate_indices.empty()) {
        return result;
    }

    // 窗口过滤：保持与 rm.cv.fans 一致，窗口内无候选时可选择直接失败（用于 indirect 回退）
    if (use_orientation_window) {
        const double window = std::max(0.0, max_orientation_angle);
        std::vector<int> in_window;
        in_window.reserve(candidate_indices.size());
        for (int idx : candidate_indices) {
            if (std::abs(predicted_z_to_v(vehicle, idx, predict_dt)) <= window) {
                in_window.push_back(idx);
            }
        }
        if (!in_window.empty()) {
            candidate_indices.swap(in_window);
        } else if (strict_orientation_window) {
            return result;
        }
    }

    const int armor_idx = choose_best_direct(
        vehicle,
        candidate_indices,
        predict_dt,
        gimbal,
        preferred_armor_idx
    );
    if (armor_idx < 0 || armor_idx >= vehicle.armor_count) {
        return result;
    }

    const auto& armor = vehicle.armors[armor_idx];
    result.valid = true;
    result.mode = AimMode::DIRECT;
    result.armor_idx = armor_idx;
    result.armor_id = armor.id;
    result.target_pos = vehicle.predict_armor_position(armor_idx, predict_dt);
    result.target_vel = compute_armor_velocity(vehicle, armor_idx, predict_dt);
    result.z_to_v = predicted_z_to_v(vehicle, armor_idx, predict_dt);
    result.time_to_fire = 0.0;
    result.armor_width = armor.width();
    result.armor_height = armor.height();
    return result;
}

ArmorAimResult ArmorAim::compute_indirect(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    const Eigen::Quaterniond* q_imu,
    double max_orientation_angle,
    double max_out_error,
    int preferred_armor_idx
) const
{
    (void)gimbal;
    (void)q_imu;

    ArmorAimResult result;
    result.mode = AimMode::INDIRECT;

    const int armor_count = vehicle.armor_count;
    if (armor_count <= 0) {
        return result;
    }

    const double omega = vehicle.spin.omega;
    if (std::abs(omega) < 1e-4) {
        return result;
    }

    const double sample_armor_width = vehicle.armors[0].width();
    const double zn_to_lim = (omega > 0.0) ? -max_orientation_angle : +max_orientation_angle;

    int best_idx = -1;
    double closest_to_lim = std::numeric_limits<double>::infinity();
    double best_armor_to_lim = 0.0;
    double best_radius = 0.0;
    double best_z_plus = 0.0;
    bool preferred_found = false;
    double preferred_armor_to_lim = std::numeric_limits<double>::infinity();
    double preferred_radius = 0.0;
    double preferred_z_plus = 0.0;

    struct CandidateDiag {
        int idx = -1;
        bool visible = false;
        double z_to_v = 0.0;
        double leave_angle = 0.0;
        double armor_to_lim = 0.0;
        double radius = 0.0;
    };
    std::vector<CandidateDiag> diags;
    diags.reserve(armor_count);

    const Eigen::Vector3d center_t = vehicle.predict_center(predict_dt);

    for (int i = 0; i < armor_count; ++i) {
        const Eigen::Vector3d armor_pos = vehicle.predict_armor_position(i, predict_dt);
        const Eigen::Vector3d offset = armor_pos - center_t;
        const double radius = std::hypot(offset.x(), offset.y());
        if (radius < 1e-5) {
            continue;
        }

        // 对齐 rm.cv.fans: max_out_angle = sample_width/2 * max_out_error / radius
        const double leave_angle = sample_armor_width * 0.5 * max_out_error / radius;
        const double z_to_v = predicted_z_to_v(vehicle, i, predict_dt);
        double armor_to_lim = (omega > 0.0)
            ? (aimer::math::reduced_angle((zn_to_lim - z_to_v) - M_PI + leave_angle) + M_PI - leave_angle)
            : (aimer::math::reduced_angle((z_to_v - zn_to_lim) - M_PI + leave_angle) + M_PI - leave_angle);

        // center-mode(top2 window ~= 0) 下，负值表示“刚刚离开中心线”。
        // 若仍把负值当作最优，会在“刚过去的板”和“下一块将出现的板”之间来回切。
        // 这里将其映射到下一圈，稳定等待将出现的板。
        if (vehicle.spin.level == predictor::SpinLevel::HIGH
            && std::abs(max_orientation_angle) <= 1e-6
            && armor_to_lim < 0.0)
        {
            armor_to_lim += 2.0 * M_PI;
        }

        diags.push_back(CandidateDiag{
            i,
            vehicle.armors[i].visible,
            z_to_v,
            leave_angle,
            armor_to_lim,
            radius
        });

        if (i == preferred_armor_idx && std::isfinite(armor_to_lim)) {
            preferred_found = true;
            preferred_armor_to_lim = armor_to_lim;
            preferred_radius = radius;
            preferred_z_plus = offset.z();
        }

        if (armor_to_lim < closest_to_lim) {
            closest_to_lim = armor_to_lim;
            best_idx = i;
            best_armor_to_lim = armor_to_lim;
            best_radius = radius;
            best_z_plus = offset.z();
        }
    }

    // 间接模式切板迟滞：当前偏好板也可用时，除非新候选显著更早进入窗口，否则保持。
    if (preferred_found
        && preferred_armor_idx >= 0
        && preferred_armor_idx < armor_count
        && best_idx >= 0
        && best_idx != preferred_armor_idx
        && std::isfinite(preferred_armor_to_lim)
        && std::isfinite(best_armor_to_lim))
    {
        const double switch_hys_deg = get_param_or(
            "AutoAim.FireControl.PID.indirect_switch_hysteresis_deg", 18.0
        );
        const double switch_hys = std::max(0.0, aimer::math::deg2rad(switch_hys_deg));
        const bool switch_not_significant =
            best_armor_to_lim + switch_hys >= preferred_armor_to_lim;
        if (switch_not_significant) {
            best_idx = preferred_armor_idx;
            best_armor_to_lim = preferred_armor_to_lim;
            best_radius = preferred_radius;
            best_z_plus = preferred_z_plus;
        }
    }

    if (best_idx < 0 || !std::isfinite(best_armor_to_lim)) {
        return result;
    }

    // 对齐 rm.cv.fans lmtd_top_model::choose_indirect_aim:
    // min_armor_to_wait 允许为负；最终瞄点是同一装甲板在
    // (predict_dt + time_to_emerge) 的预测位置，而不是强制投影到 lim 方向。
    const double time_to_emerge = best_armor_to_lim / std::abs(omega);
    const Eigen::Vector3d emerge_pos =
        vehicle.predict_armor_position(best_idx, predict_dt + time_to_emerge);
    if (!emerge_pos.allFinite() || emerge_pos.squaredNorm() < 1e-9) {
        return ArmorAimResult{};
    }

    result.valid = true;
    result.armor_idx = best_idx;
    result.armor_id = vehicle.armors[best_idx].id;
    result.target_pos = emerge_pos;
    // 与 rm.cv.fans 一致：返回被选中装甲板在当前时刻的 z_to_v，
    // 后续开火门控会基于各自时刻再次计算。
    result.z_to_v = predicted_z_to_v(vehicle, best_idx, predict_dt);
    result.time_to_fire = time_to_emerge;
    result.armor_width = vehicle.armors[best_idx].width();
    result.armor_height = vehicle.armors[best_idx].height();

    // 对齐 rm.cv.fans: indirect 的 ypd_v 仅使用 center_v。
    result.target_vel = vehicle.velocity;

    if (get_param_or("AutoAim.FireControl.Debug.indirect_detail", false)) {
        static double last_diag_log_sec = 0.0;
        static int last_diag_best_idx = -1;
        const double period_s = std::max(
            0.05, get_param_or("AutoAim.FireControl.Debug.indirect_period_s", 0.2)
        );
        const double now = now_sec();
        const bool best_switched = (best_idx != last_diag_best_idx);
        if (best_switched || (now - last_diag_log_sec) >= period_s) {
            debug::print(
                best_switched ? debug::PrintMode::INFO : debug::PrintMode::DEBUG,
                "ArmorAim",
                "[INDIRECT] best={} pref={} omega={:.2f}rad/s lim={:.1f}deg "
                "ttf={:.1f}ms pred_dt={:.1f}ms r={:.3f}m z+={:.3f} z2v_now={:.1f}deg",
                best_idx,
                preferred_armor_idx,
                omega,
                aimer::math::rad2deg(zn_to_lim),
                time_to_emerge * 1000.0,
                predict_dt * 1000.0,
                best_radius,
                best_z_plus,
                aimer::math::rad2deg(result.z_to_v)
            );

            std::string line;
            for (const auto& d : diags) {
                line += fmt::format(
                    " i{}(vis={},z2v={:.1f},leave={:.1f},alim={:.1f},t={:.1f}ms,r={:.3f})",
                    d.idx,
                    d.visible ? "Y" : "N",
                    aimer::math::rad2deg(d.z_to_v),
                    aimer::math::rad2deg(d.leave_angle),
                    aimer::math::rad2deg(d.armor_to_lim),
                    d.armor_to_lim / std::abs(omega) * 1000.0,
                    d.radius
                );
            }
            debug::print(debug::PrintMode::DEBUG, "ArmorAim", "[INDIRECT-CANDS]{}", line);

            last_diag_log_sec = now;
            last_diag_best_idx = best_idx;
        }
    }

    return result;
}

int ArmorAim::choose_best_direct(
    const predictor::VehicleState& vehicle,
    const std::vector<int>& direct_indices,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int preferred_armor_idx
) const
{
    if (direct_indices.empty()) {
        return -1;
    }

    auto score_idx = [&](int idx) {
        if (gimbal != nullptr) {
            const Eigen::Vector3d pos = vehicle.predict_armor_position(idx, predict_dt);
            return center_cost(pos, *gimbal);
        }
        // 无云台状态时回退到“最正对”策略
        return std::abs(predicted_z_to_v(vehicle, idx, predict_dt));
    };

    int best_idx = direct_indices[0];
    double best_score = score_idx(best_idx);

    for (int idx : direct_indices) {
        const double score = score_idx(idx);
        if (score < best_score) {
            best_score = score;
            best_idx = idx;
        }
    }

    // 切板迟滞：上一帧板子仍在候选内且与当前最优差距不大时，继续保持。
    // 语义对齐 rm.cv.fans “先保持追踪，再在优势明显时切换”。
    // 默认切板迟滞
    double switch_hys_deg = get_param_or(
        "AutoAim.FireControl.PID.switch_armor_hysteresis_deg", 3.0
    );
    // 超快陀螺 + center-mode（top2 window=0）时，增强迟滞抑制 0/3 频繁切板
    // 该模式下不走窗口/indirect，直接瞄中心，过快切板会导致橙黄点大跳。
    if (vehicle.spin.active && vehicle.spin.level == predictor::SpinLevel::HIGH) {
        const double top2_window_deg = get_param_or(
            "AutoAim.FireControl.PID.top2_max_orientation_angle", 0.0
        );
        if (std::abs(top2_window_deg) <= 1e-6) {
            const double center_hys_deg = get_param_or(
                "AutoAim.FireControl.PID.center_mode_switch_hysteresis_deg", 18.0
            );
            switch_hys_deg = std::max(switch_hys_deg, center_hys_deg);
        }
    }
    const double switch_hys = std::max(0.0, aimer::math::deg2rad(switch_hys_deg));
    if (preferred_armor_idx >= 0
        && preferred_armor_idx < vehicle.armor_count
        && std::find(direct_indices.begin(), direct_indices.end(), preferred_armor_idx) != direct_indices.end())
    {
        const double preferred_score = score_idx(preferred_armor_idx);
        if (preferred_score <= best_score + switch_hys) {
            return preferred_armor_idx;
        }
    }

    return best_idx;
}

Eigen::Vector3d ArmorAim::compute_armor_velocity(
    const predictor::VehicleState& vehicle,
    int armor_idx,
    double predict_dt
) const
{
    if (armor_idx < 0 || armor_idx >= vehicle.armor_count) {
        return Eigen::Vector3d::Zero();
    }

    if (!vehicle.spin.active) {
        return vehicle.armors[armor_idx].velocity;
    }

    // 陀螺模式: v = v_center + ω × (armor_pos - center)
    double omega = vehicle.spin.omega;
    const Eigen::Vector3d center = vehicle.predict_center(predict_dt);
    const Eigen::Vector3d armor_pos = vehicle.predict_armor_position(armor_idx, predict_dt);
    Eigen::Vector3d offset = armor_pos - center;
    Eigen::Vector3d tangent_vel(
        -omega * offset.y(),
        +omega * offset.x(),
        0
    );

    // 加上中心速度
    return vehicle.velocity + tangent_vel;
}

}  // namespace autoaim::fire_control
