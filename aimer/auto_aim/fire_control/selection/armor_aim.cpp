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

bool is_direct_candidate(const predictor::ArmorState& armor, double max_orientation_angle)
{
    return armor.visible && (std::abs(armor.z_to_v) < max_orientation_angle);
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

    // 根据陀螺状态选择瞄准方式
    if (!vehicle.spin.active || std::abs(vehicle.spin.omega) < 0.5) {
        // 非陀螺: 直接跟踪
        return compute_non_spin(vehicle, predict_dt, gimbal, preferred_armor_idx);
    } else {
        // 陀螺: DIRECT 或 INDIRECT
        return compute_spin(vehicle, predict_dt, gimbal, preferred_armor_idx);
    }
}

ArmorAimResult ArmorAim::compute_non_spin(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int preferred_armor_idx
) const
{
    ArmorAimResult result;
    result.mode = AimMode::DIRECT;

    std::vector<int> direct_indices;
    direct_indices.reserve(vehicle.armor_count);
    for (int i = 0; i < vehicle.armor_count; ++i) {
        // 非陀螺路径: 不使用 max_orientation_angle 过滤，避免调反陀螺阈值影响常规跟踪。
        if (vehicle.armors[i].visible) {
            direct_indices.push_back(i);
        }
    }
    if (direct_indices.empty()) {
        // 非陀螺模式不做盲打预测，直接判 invalid
        return result;
    }

    const int armor_idx = choose_best_direct(
        vehicle, direct_indices, predict_dt, gimbal, preferred_armor_idx
    );
    if (armor_idx < 0 || armor_idx >= vehicle.armor_count) {
        return result;
    }

    const auto& armor = vehicle.armors[armor_idx];

    result.valid = true;
    result.armor_idx = armor_idx;
    result.target_pos = vehicle.predict_armor_position(armor_idx, predict_dt);
    result.target_vel = armor.velocity;
    result.z_to_v = armor.z_to_v;
    result.armor_width = armor.width();
    result.armor_height = armor.height();

    return result;
}

ArmorAimResult ArmorAim::compute_spin(
    const predictor::VehicleState& vehicle,
    double predict_dt,
    const ::fire_control::GimbalState* gimbal,
    int preferred_armor_idx
) const
{
    // 读取参数 (每次调用都从 runtime_param 获取，支持热更新)
    const double max_orientation_angle = runtime_param::get_param<double>(
        "AutoAim.FireControl.PID.max_orientation_angle"
    ) * M_PI / 180.0;

    // 1. 收集 DIRECT 候选（仅可见 + 朝向可打）
    std::vector<int> direct_indices;
    for (int i = 0; i < vehicle.armor_count; ++i) {
        if (is_direct_candidate(vehicle.armors[i], max_orientation_angle)) {
            direct_indices.push_back(i);
        }
    }

    if (!direct_indices.empty()) {
        // === DIRECT 模式 ===
        ArmorAimResult result;
        result.valid = true;
        result.mode = AimMode::DIRECT;

        // 选择最佳可见装甲板（喵中心最小移动 + 切板迟滞）
        result.armor_idx = choose_best_direct(
            vehicle, direct_indices, predict_dt, gimbal, preferred_armor_idx
        );
        if (result.armor_idx < 0 || result.armor_idx >= vehicle.armor_count) {
            return ArmorAimResult{};
        }
        const auto& armor = vehicle.armors[result.armor_idx];

        // 预测位置 (考虑陀螺旋转)
        result.target_pos = vehicle.predict_armor_position(result.armor_idx, predict_dt);
        result.target_vel = compute_armor_velocity(vehicle, result.armor_idx);
        result.z_to_v = armor.z_to_v;
        result.armor_width = armor.width();
        result.armor_height = armor.height();

        return result;

    } else {
        // === INDIRECT 模式 ===
        return compute_indirect(vehicle, predict_dt);
    }
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

    // 默认权重: 以喵中心最小移动为主，朝向角仅作次级约束
    const double orient_weight = get_param_or(
        "AutoAim.FireControl.PID.direct_orientation_weight", 0.15
    );
    const double switch_hysteresis = std::max(0.0, get_param_or(
        "AutoAim.FireControl.PID.switch_armor_hysteresis", 0.12
    ));

    auto score_idx = [&](int idx) {
        const auto& armor = vehicle.armors[idx];
        double score = std::abs(armor.z_to_v) * orient_weight;
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

ArmorAimResult ArmorAim::compute_indirect(
    const predictor::VehicleState& vehicle,
    double predict_dt
) const
{
    ArmorAimResult result;
    result.mode = AimMode::INDIRECT;

    // 读取参数 (每次调用都从 runtime_param 获取，支持热更新)
    const double max_orientation_angle = runtime_param::get_param<double>(
        "AutoAim.FireControl.PID.max_orientation_angle"
    ) * M_PI / 180.0;

    double omega = vehicle.spin.omega;
    const double abs_omega = std::abs(omega);
    int armor_count = vehicle.armor_count;
    if (armor_count <= 0 || abs_omega < 1e-6) {
        return result;
    }

    // 陀螺旋转方向: omega>0 为逆时针
    bool ccw = (omega > 0);

    // 出现边界角: 与 rm.cv.fans 保持同向定义
    double target_z_to_v = ccw ? -max_orientation_angle : max_orientation_angle;

    int best_armor = -1;
    double min_time_to_emerge = std::numeric_limits<double>::max();

    for (int i = 0; i < armor_count; ++i) {
        double armor_z_to_v = vehicle.armors[i].z_to_v;

        // 沿旋转方向计算“前向角距离”:
        // CCW: target - current
        // CW : current - target
        double angle_diff = ccw
            ? aimer::math::reduced_angle(target_z_to_v - armor_z_to_v)
            : aimer::math::reduced_angle(armor_z_to_v - target_z_to_v);
        if (angle_diff < 0) angle_diff += 2 * M_PI;

        // 转换为时间
        double time_to_emerge = angle_diff / abs_omega;

        // 选择最快出现的装甲板
        if (time_to_emerge < min_time_to_emerge) {
            min_time_to_emerge = time_to_emerge;
            best_armor = i;
        }
    }

    if (best_armor < 0) {
        return result;
    }

    result.valid = true;
    result.armor_idx = best_armor;
    result.time_to_fire = min_time_to_emerge;

    // 计算"出现位置" (emerging position)
    // = 装甲板在 time_to_emerge + predict_dt 后的位置
    double total_dt = min_time_to_emerge + predict_dt;
    result.target_pos = vehicle.predict_armor_position(best_armor, total_dt);
    result.target_vel = compute_armor_velocity(vehicle, best_armor);
    result.z_to_v = target_z_to_v;  // 边界角度
    result.armor_width = vehicle.armors[best_armor].width();
    result.armor_height = vehicle.armors[best_armor].height();

    return result;
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
