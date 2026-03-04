/**
 * @file armor_aim.cpp
 * @brief 装甲板瞄准逻辑实现
 */

#include "armor_aim.hpp"

#include <algorithm>
#include <cmath>

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

double orientation_window_penalty(
    double z_to_v,
    bool use_orientation_window,
    double max_orientation_angle
)
{
    if (!use_orientation_window || max_orientation_angle <= 0.0) {
        return 0.0;
    }
    const double abs_z = std::abs(z_to_v);
    if (abs_z <= max_orientation_angle) {
        return 0.0;
    }
    return abs_z - max_orientation_angle;
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
    int preferred_armor_idx
) const
{
    // 非陀螺: 仅按可见板 direct-center，不使用 orientation 窗口
    return compute_direct_visible(
        vehicle, predict_dt, gimbal, preferred_armor_idx,
        /*use_orientation_window=*/false,
        /*max_orientation_angle=*/0.0
    );
}

ArmorAimResult ArmorAim::compute_spin(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int preferred_armor_idx
) const
{
    // 陀螺: 可见板 direct-center，orientation 窗口仅作软偏好
    const double max_orientation_angle = runtime_param::get_param<double>(
        "AutoAim.FireControl.PID.max_orientation_angle"
    ) * M_PI / 180.0;
    const bool use_orientation_window = max_orientation_angle > 0.0;

    return compute_direct_visible(
        vehicle, predict_dt, gimbal, preferred_armor_idx,
        use_orientation_window, max_orientation_angle
    );
}

ArmorAimResult ArmorAim::compute_direct_visible(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int preferred_armor_idx,
    bool use_orientation_window,
    double max_orientation_angle
) const
{
    ArmorAimResult result;
    result.mode = AimMode::DIRECT;

    std::vector<int> direct_indices;
    direct_indices.reserve(vehicle.armor_count);
    for (int i = 0; i < vehicle.armor_count; ++i) {
        if (vehicle.armors[i].visible) {
            direct_indices.push_back(i);
        }
    }
    if (direct_indices.empty()) {
        return result;
    }

    const int armor_idx = choose_best_direct(
        vehicle,
        direct_indices,
        predict_dt,
        gimbal,
        preferred_armor_idx,
        use_orientation_window,
        max_orientation_angle
    );
    if (armor_idx < 0 || armor_idx >= vehicle.armor_count) {
        return result;
    }

    const auto& armor = vehicle.armors[armor_idx];
    result.valid = true;
    result.mode = AimMode::DIRECT;
    result.armor_idx = armor_idx;
    result.target_pos = vehicle.predict_armor_position(armor_idx, predict_dt);
    result.target_vel = compute_armor_velocity(vehicle, armor_idx);
    result.z_to_v = armor.z_to_v;
    result.time_to_fire = 0.0;
    result.armor_width = armor.width();
    result.armor_height = armor.height();
    return result;
}

int ArmorAim::choose_best_direct(
    const predictor::VehicleState& vehicle,
    const std::vector<int>& direct_indices,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int preferred_armor_idx,
    bool use_orientation_window,
    double max_orientation_angle
) const
{
    if (direct_indices.empty()) {
        return -1;
    }

    // 默认权重: 以喵中心最小移动为主，朝向角仅作次级约束
    const double orient_weight = get_param_or(
        "AutoAim.FireControl.PID.direct_orientation_weight", 0.15
    );
    const double switch_hysteresis = std::max(0.0, get_param_or(
        "AutoAim.FireControl.PID.switch_armor_hysteresis", 0.12
    ));

    auto score_idx = [&](int idx) {
        const auto& armor = vehicle.armors[idx];
        double score = orient_weight * orientation_window_penalty(
            armor.z_to_v, use_orientation_window, max_orientation_angle
        );
        if (gimbal != nullptr) {
            const Eigen::Vector3d pos = vehicle.predict_armor_position(idx, predict_dt);
            score += center_cost(pos, *gimbal);
        } else {
            // 无云台状态时回退到“最正对”策略
            score += std::abs(armor.z_to_v);
        }
        return score;
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

    // 切板迟滞: 上一板仍在可打集合且并不明显更差时，优先保持
    const bool preferred_in_set = std::find(
        direct_indices.begin(), direct_indices.end(), preferred_armor_idx
    ) != direct_indices.end();
    if (preferred_in_set) {
        const double preferred_score = score_idx(preferred_armor_idx);
        if (preferred_score <= best_score * (1.0 + switch_hysteresis)) {
            return preferred_armor_idx;
        }
    }

    return best_idx;
}

Eigen::Vector3d ArmorAim::compute_armor_velocity(
    const predictor::VehicleState& vehicle,
    int armor_idx
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
    Eigen::Vector3d offset = vehicle.armors[armor_idx].position - vehicle.center;
    Eigen::Vector3d tangent_vel(
        -omega * offset.y(),
        +omega * offset.x(),
        0
    );

    // 加上中心速度
    return vehicle.velocity + tangent_vel;
}

}  // namespace autoaim::fire_control
