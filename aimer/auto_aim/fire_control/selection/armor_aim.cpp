/**
 * @file armor_aim.cpp
 * @brief 装甲板瞄准逻辑实现
 */

#include "armor_aim.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "aimer/common/math/math.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

namespace {

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

}  // namespace

ArmorAimResult ArmorAim::compute(
    const predictor::VehicleState& vehicle,
    double predict_dt
) const
{
    return compute(vehicle, predict_dt, nullptr, -1);
}

ArmorAimResult ArmorAim::compute(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int preferred_armor_idx
) const
{
    if (!vehicle.valid) {
        return ArmorAimResult{};
    }

    // 是否陀螺仅信 predictor 输出，避免火控重复阈值判定
    if (!vehicle.spin.active) {
        return compute_non_spin(vehicle, predict_dt, gimbal, preferred_armor_idx);
    }
    return compute_spin(vehicle, predict_dt, gimbal, preferred_armor_idx);
}

ArmorAimResult ArmorAim::compute_non_spin(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int /* preferred_armor_idx */
) const
{
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
    int preferred_armor_idx
) const
{
    // 陀螺:
    // - max_orientation_angle <= 0: 强制 direct-center
    // - 高速陀螺: 强制 direct-center
    // - 普通陀螺: direct(窗口内) -> indirect(待出现点)
    const double max_orientation_angle = runtime_param::get_param<double>(
        "AutoAim.FireControl.PID.max_orientation_angle"
    ) * M_PI / 180.0;

    const bool force_direct_center = (max_orientation_angle <= 0.0)
        || (vehicle.spin.level == predictor::SpinLevel::HIGH);

    if (force_direct_center) {
        return compute_direct(
            vehicle, predict_dt, gimbal, preferred_armor_idx,
            /*use_orientation_window=*/false,
            /*max_orientation_angle=*/0.0,
            /*visible_only=*/true,
            /*strict_orientation_window=*/false
        );
    }

    // rm.cv.fans: 先 direct（窗口内）
    ArmorAimResult direct = compute_direct(
        vehicle, predict_dt, gimbal, preferred_armor_idx,
        /*use_orientation_window=*/true,
        max_orientation_angle,
        /*visible_only=*/false,
        /*strict_orientation_window=*/true
    );
    if (direct.valid) {
        return direct;
    }

    // rm.cv.fans: direct 失败后用 indirect（等待板进入窗口）
    ArmorAimResult indirect = compute_indirect(
        vehicle, predict_dt, gimbal, max_orientation_angle
    );
    if (indirect.valid) {
        return indirect;
    }

    // 保底：仍无法间接求解时回退到可见 direct-center
    return compute_direct(
        vehicle, predict_dt, gimbal, preferred_armor_idx,
        /*use_orientation_window=*/false,
        /*max_orientation_angle=*/0.0,
        /*visible_only=*/true,
        /*strict_orientation_window=*/false
    );
}

ArmorAimResult ArmorAim::compute_direct(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int /* preferred_armor_idx */,
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
    if (use_orientation_window && max_orientation_angle > 0.0) {
        std::vector<int> in_window;
        in_window.reserve(candidate_indices.size());
        for (int idx : candidate_indices) {
            if (std::abs(predicted_z_to_v(vehicle, idx, predict_dt)) <= max_orientation_angle) {
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
        gimbal
    );
    if (armor_idx < 0 || armor_idx >= vehicle.armor_count) {
        return result;
    }

    const auto& armor = vehicle.armors[armor_idx];
    result.valid = true;
    result.mode = AimMode::DIRECT;
    result.armor_idx = armor_idx;
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
    double max_orientation_angle
) const
{
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

    const double max_out_error = std::max(0.0, get_param_or(
        "AutoAim.FireControl.PID.max_out_error", 0.2
    ));
    const double zn_to_lim = (omega > 0.0) ? -max_orientation_angle : +max_orientation_angle;

    int best_idx = -1;
    double closest_to_lim = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d best_armor_pos = Eigen::Vector3d::Zero();
    double best_armor_to_lim = 0.0;

    const Eigen::Vector3d center_t = vehicle.predict_center(predict_dt);

    for (int i = 0; i < armor_count; ++i) {
        const Eigen::Vector3d armor_pos = vehicle.predict_armor_position(i, predict_dt);
        const Eigen::Vector3d offset = armor_pos - center_t;
        const double radius = std::hypot(offset.x(), offset.y());
        if (radius < 1e-5) {
            continue;
        }

        // 与 rm.cv.fans 保持同义：允许“离场角”修正，避免窗口边界误触发
        const double leave_angle = std::clamp(
            vehicle.armors[i].width() * 0.5 * max_out_error / radius,
            0.0,
            M_PI * 0.45
        );
        const double z_to_v = predicted_z_to_v(vehicle, i, predict_dt);
        const double armor_to_lim = (omega > 0.0)
            ? (aimer::math::reduced_angle((zn_to_lim - z_to_v) - M_PI + leave_angle) + M_PI - leave_angle)
            : (aimer::math::reduced_angle((z_to_v - zn_to_lim) - M_PI + leave_angle) + M_PI - leave_angle);

        if (armor_to_lim < closest_to_lim) {
            closest_to_lim = armor_to_lim;
            best_idx = i;
            best_center = center_t;
            best_armor_pos = armor_pos;
            best_armor_to_lim = armor_to_lim;
        }
    }

    if (best_idx < 0 || !std::isfinite(best_armor_to_lim)) {
        return result;
    }

    const double time_to_emerge = std::max(0.0, best_armor_to_lim / std::abs(omega));
    const Eigen::Vector3d center_future = vehicle.predict_center(predict_dt + time_to_emerge);
    const Eigen::Vector3d offset_t = best_armor_pos - best_center;

    const double delta = omega * time_to_emerge;
    const double c = std::cos(delta);
    const double s = std::sin(delta);
    Eigen::Vector3d offset_rot(
        offset_t.x() * c - offset_t.y() * s,
        offset_t.x() * s + offset_t.y() * c,
        offset_t.z()
    );
    const Eigen::Vector3d emerge_pos = center_future + offset_rot;

    result.valid = true;
    result.armor_idx = best_idx;
    result.target_pos = emerge_pos;
    result.z_to_v = zn_to_lim;
    result.time_to_fire = time_to_emerge;
    result.armor_width = vehicle.armors[best_idx].width();
    result.armor_height = vehicle.armors[best_idx].height();

    // 速度用于前馈：中心速度 + 切向速度
    Eigen::Vector3d tangent_vel(
        -omega * offset_rot.y(),
        +omega * offset_rot.x(),
        0.0
    );
    result.target_vel = vehicle.velocity + tangent_vel;

    // gimbal 仅用于 debug 语义，保持参数位兼容，避免未使用告警
    (void)gimbal;
    return result;
}

int ArmorAim::choose_best_direct(
    const predictor::VehicleState& vehicle,
    const std::vector<int>& direct_indices,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal
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
