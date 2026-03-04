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

double projected_area_proxy(
    const predictor::VehicleState& vehicle,
    int armor_idx,
    double predict_dt
)
{
    if (armor_idx < 0 || armor_idx >= vehicle.armor_count) {
        return 0.0;
    }

    const auto& armor = vehicle.armors[armor_idx];
    const Eigen::Vector3d pos = vehicle.predict_armor_position(armor_idx, predict_dt);
    const double dist2 = std::max(1e-6, pos.squaredNorm());
    const double facing = std::abs(std::cos(armor.z_to_v));
    const double geom_area = armor.width() * armor.height();

    // 火控阶段无法直接访问检测像素面积，使用投影面积代理:
    // A_proj ∝ A_geom * cos(z_to_v) / d^2
    return geom_area * facing / dist2;
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
    // 陀螺:
    // - 高速陀螺: 强制可见板喵中心
    // - 低速陀螺: orientation 窗口硬筛选（无候选时回退可见板喵中心）
    const double max_orientation_angle = runtime_param::get_param<double>(
        "AutoAim.FireControl.PID.max_orientation_angle"
    ) * M_PI / 180.0;
    const bool high_spin = (vehicle.spin.level == predictor::SpinLevel::HIGH);
    const bool use_orientation_window = (!high_spin) && (max_orientation_angle > 0.0);

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

    std::vector<int> visible_indices;
    visible_indices.reserve(vehicle.armor_count);
    for (int i = 0; i < vehicle.armor_count; ++i) {
        if (vehicle.armors[i].visible) {
            visible_indices.push_back(i);
        }
    }
    if (visible_indices.empty()) {
        return result;
    }

    // 低速陀螺时，先尝试在 orientation 窗口内选可见板。
    // 若窗口内无可见板（例如英雄该板尚未转入可见区），回退到全部可见板喵中心。
    std::vector<int> candidate_indices = visible_indices;
    if (use_orientation_window && max_orientation_angle > 0.0) {
        std::vector<int> in_window;
        in_window.reserve(visible_indices.size());
        for (int idx : visible_indices) {
            if (std::abs(vehicle.armors[idx].z_to_v) <= max_orientation_angle) {
                in_window.push_back(idx);
            }
        }
        if (!in_window.empty()) {
            candidate_indices.swap(in_window);
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
    int preferred_armor_idx
) const
{
    if (direct_indices.empty()) {
        return -1;
    }

    const double keep_tracking_area_ratio = std::clamp(get_param_or(
        "AutoAim.FireControl.PID.keep_tracking_area_ratio", 0.9
    ), 0.0, 1.0);

    auto score_idx = [&](int idx) {
        if (gimbal != nullptr) {
            const Eigen::Vector3d pos = vehicle.predict_armor_position(idx, predict_dt);
            return center_cost(pos, *gimbal);
        }
        // 无云台状态时回退到“最正对”策略
        const auto& armor = vehicle.armors[idx];
        return std::abs(armor.z_to_v);
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

    // 按 rm.cv.fans keep-tracking-area-ratio 思路:
    // 仅当上一板的“投影面积代理”不小于最大值的指定比例时才保持。
    const bool preferred_in_set = std::find(
        direct_indices.begin(), direct_indices.end(), preferred_armor_idx
    ) != direct_indices.end();
    if (preferred_in_set) {
        double max_area_proxy = 0.0;
        for (int idx : direct_indices) {
            max_area_proxy = std::max(
                max_area_proxy,
                projected_area_proxy(vehicle, idx, predict_dt)
            );
        }
        const double preferred_area_proxy =
            projected_area_proxy(vehicle, preferred_armor_idx, predict_dt);
        if (preferred_area_proxy >= keep_tracking_area_ratio * max_area_proxy) {
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
