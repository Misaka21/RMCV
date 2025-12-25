/**
 * @file spin_aim.cpp
 * @brief 反陀螺瞄准逻辑实现
 */

#include "spin_aim.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "aimer/common/math/math.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

SpinAimResult SpinAim::compute(
    const predictor::VehicleState& vehicle,
    double predict_dt
) const
{
    if (!vehicle.valid) {
        return SpinAimResult{};
    }

    // 根据陀螺状态选择瞄准方式
    if (!vehicle.spin.active || std::abs(vehicle.spin.omega) < 0.5) {
        // 非陀螺: 直接跟踪
        return compute_non_spin(vehicle, predict_dt);
    } else {
        // 陀螺: DIRECT 或 INDIRECT
        return compute_spin(vehicle, predict_dt);
    }
}

SpinAimResult SpinAim::compute_non_spin(
    const predictor::VehicleState& vehicle,
    double predict_dt
) const
{
    SpinAimResult result;
    result.mode = AimMode::DIRECT;

    // 使用推荐装甲板
    const auto* armor = vehicle.get_recommended_armor();
    if (!armor) {
        return result;
    }

    result.valid = true;
    result.armor_idx = armor->id;
    result.target_pos = armor->predict_position(predict_dt);
    result.target_vel = armor->velocity;
    result.z_to_v = armor->z_to_v;

    return result;
}

SpinAimResult SpinAim::compute_spin(
    const predictor::VehicleState& vehicle,
    double predict_dt
) const
{
    // 读取参数
    max_orientation_angle_ = runtime_param::get_param<double>(
        "AutoAim.FireControl.PID.max_orientation_angle"
    ) * M_PI / 180.0;

    // 1. 收集可见装甲板 (|z_to_v| < max_orientation_angle)
    std::vector<int> direct_indices;
    for (int i = 0; i < vehicle.armor_count; ++i) {
        double z_to_v = std::abs(vehicle.armors[i].z_to_v);
        if (z_to_v < max_orientation_angle_) {
            direct_indices.push_back(i);
        }
    }

    if (!direct_indices.empty()) {
        // === DIRECT 模式 ===
        SpinAimResult result;
        result.valid = true;
        result.mode = AimMode::DIRECT;

        // 选择最佳可见装甲板
        result.armor_idx = choose_best_direct(vehicle, direct_indices);
        const auto& armor = vehicle.armors[result.armor_idx];

        // 预测位置 (考虑陀螺旋转)
        result.target_pos = vehicle.predict_armor_position(result.armor_idx, predict_dt);
        result.target_vel = compute_armor_velocity(vehicle, result.armor_idx);
        result.z_to_v = armor.z_to_v;

        return result;

    } else {
        // === INDIRECT 模式 ===
        return compute_indirect(vehicle, predict_dt);
    }
}

int SpinAim::choose_best_direct(
    const predictor::VehicleState& vehicle,
    const std::vector<int>& direct_indices
) const
{
    // 选择朝向最正的装甲板 (|z_to_v| 最小)
    int best_idx = direct_indices[0];
    double min_z_to_v = std::abs(vehicle.armors[best_idx].z_to_v);

    for (int idx : direct_indices) {
        double z_to_v = std::abs(vehicle.armors[idx].z_to_v);
        if (z_to_v < min_z_to_v) {
            min_z_to_v = z_to_v;
            best_idx = idx;
        }
    }

    return best_idx;
}

SpinAimResult SpinAim::compute_indirect(
    const predictor::VehicleState& vehicle,
    double predict_dt
) const
{
    SpinAimResult result;
    result.mode = AimMode::INDIRECT;

    double omega = vehicle.spin.omega;
    double theta = vehicle.spin.phase;
    int armor_count = vehicle.armor_count;

    // 陀螺旋转方向
    bool ccw = (omega > 0);  // 逆时针

    // 计算每块装甲板到"出现位置"的角度差
    // 出现位置 = 装甲板朝向角达到 ±max_orientation_angle 时
    double target_z_to_v = ccw ? -max_orientation_angle_ : max_orientation_angle_;

    int best_armor = -1;
    double min_time_to_emerge = std::numeric_limits<double>::max();

    for (int i = 0; i < armor_count; ++i) {
        double armor_z_to_v = vehicle.armors[i].z_to_v;

        // 计算该装甲板旋转到 target_z_to_v 需要的角度
        double angle_diff;
        if (ccw) {
            // 逆时针: 装甲板 z_to_v 减小
            angle_diff = armor_z_to_v - target_z_to_v;
            if (angle_diff < 0) angle_diff += 2 * M_PI;
        } else {
            // 顺时针: 装甲板 z_to_v 增大
            angle_diff = target_z_to_v - armor_z_to_v;
            if (angle_diff < 0) angle_diff += 2 * M_PI;
        }

        // 转换为时间
        double time_to_emerge = angle_diff / std::abs(omega);

        // 选择最快出现的装甲板
        if (time_to_emerge < min_time_to_emerge && time_to_emerge > 0.01) {
            min_time_to_emerge = time_to_emerge;
            best_armor = i;
        }
    }

    if (best_armor < 0) {
        // 没找到合适的装甲板，回退到第一块
        best_armor = 0;
        min_time_to_emerge = 0;
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

    return result;
}

Eigen::Vector3d SpinAim::compute_armor_velocity(
    const predictor::VehicleState& vehicle,
    int armor_idx
) const
{
    if (!vehicle.spin.active) {
        return vehicle.armors[armor_idx].velocity;
    }

    // 陀螺模式: v = ω × r
    double omega = vehicle.spin.omega;
    double r = (armor_idx % 2 == 0) ? vehicle.spin.radius : vehicle.spin.radius_2;

    // 装甲板相对于中心的位置角度
    double armor_angle = vehicle.spin.phase + armor_idx * (2.0 * M_PI / vehicle.armor_count);

    // 切向速度 (垂直于半径方向)
    // v_x = -ω * r * sin(θ)
    // v_y = +ω * r * cos(θ)
    Eigen::Vector3d tangent_vel(
        -omega * r * std::sin(armor_angle),
        +omega * r * std::cos(armor_angle),
        0
    );

    // 加上中心速度
    return vehicle.velocity + tangent_vel;
}

}  // namespace autoaim::fire_control
