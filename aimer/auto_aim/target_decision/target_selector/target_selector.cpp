/**
 * @file target_selector.cpp
 * @brief 目标选择器实现
 *
 * 选择策略 (参考 rm.cv.fans):
 *   1. 选敌人：最靠近图像中心 (操作手意图)
 *   2. 选装甲板：电机转动代价最小 (swing_cost)
 *
 * 锁定机制:
 *   - 当有目标且可见时，保持当前目标
 *   - 当目标丢失超过 keep_time 后才切换
 */

#include "target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

// ============================================================================
// 主选择函数
// ============================================================================

TargetSelection TargetSelector::select(
    const predictor::BattlefieldSnapshot& snapshot,
    const GimbalState& gimbal,
    double dt)
{
    TargetSelection result;
    double current_time = snapshot.timestamp;

    // 读取参数
    double keep_time = runtime_param::get_param<double>(
        "AutoAim.FireControl.TargetSelector.keep_time"
    );
    double max_angle = runtime_param::get_param<double>(
        "AutoAim.FireControl.TargetSelector.max_angle"
    );

    // ========== 1. 预瞄锁定优先 ==========
    if (forced_target_id_ >= 0 && snapshot.is_valid(forced_target_id_)) {
        const auto& vehicle = snapshot.vehicles[forced_target_id_];
        int armor_idx = select_best_armor(vehicle, gimbal, dt);

        if (armor_idx >= 0) {
            const auto& armor = vehicle.armors[armor_idx];
            result.has_target = true;
            result.target_id = forced_target_id_;
            result.armor_idx = armor_idx;
            result.vehicle = &vehicle;
            result.armor = &armor;
            result.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);

            current_target_id_ = forced_target_id_;
            current_armor_idx_ = armor_idx;
            last_seen_time_ = current_time;
            return result;
        }
    }

    // ========== 2. 检查当前目标是否还有效 ==========
    bool current_target_valid = false;
    if (current_target_id_ >= 0 && snapshot.is_valid(current_target_id_)) {
        const auto& vehicle = snapshot.vehicles[current_target_id_];
        int armor_idx = select_best_armor(vehicle, gimbal, dt);

        if (armor_idx >= 0) {
            current_target_valid = true;
            last_seen_time_ = current_time;

            // 当前目标仍然有效，保持追踪
            const auto& armor = vehicle.armors[armor_idx];
            result.has_target = true;
            result.target_id = current_target_id_;
            result.armor_idx = armor_idx;
            result.vehicle = &vehicle;
            result.armor = &armor;
            result.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);

            current_armor_idx_ = armor_idx;
        }
    }

    // ========== 3. 如果当前目标有效，检查是否在保持期内 ==========
    if (current_target_valid) {
        return result;  // 继续追踪当前目标
    }

    // 当前目标无效，检查是否在保持期内
    if (current_target_id_ >= 0 && (current_time - last_seen_time_) < keep_time) {
        // 在保持期内，不切换目标，返回空结果
        // 这样 FireController 会使用上次的预测继续追踪
        return result;
    }

    // ========== 4. 需要选择新目标：选最靠近中心的敌人 ==========
    int best_target_id = -1;
    double min_center_dist = std::numeric_limits<double>::infinity();

    snapshot.for_each_valid([&](int id, const predictor::VehicleState& vehicle) {
        if (!vehicle.valid || vehicle.confidence < 0.1) return;

        // 找该敌人最近中心的装甲板
        for (int i = 0; i < vehicle.armor_count; ++i) {
            const auto& armor = vehicle.armors[i];

            // 跳过不可见的 (陀螺模式除外)
            if (!armor.visible && !vehicle.spin.active) continue;

            // 角度过滤
            if (std::abs(armor.z_to_v) > max_angle) continue;

            // 计算到中心的距离
            Eigen::Vector3d pos = vehicle.predict_armor_position(i, dt);
            double dist = compute_center_distance(pos, gimbal);

            if (dist < min_center_dist) {
                min_center_dist = dist;
                best_target_id = id;
            }
        }
    });

    // ========== 5. 找到了新目标 ==========
    if (best_target_id >= 0) {
        const auto& vehicle = snapshot.vehicles[best_target_id];
        int armor_idx = select_best_armor(vehicle, gimbal, dt);

        if (armor_idx >= 0) {
            const auto& armor = vehicle.armors[armor_idx];
            result.has_target = true;
            result.target_id = best_target_id;
            result.armor_idx = armor_idx;
            result.vehicle = &vehicle;
            result.armor = &armor;
            result.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);

            current_target_id_ = best_target_id;
            current_armor_idx_ = armor_idx;
            last_seen_time_ = current_time;
        }
    } else {
        // 没有目标
        current_target_id_ = -1;
        current_armor_idx_ = -1;
    }

    return result;
}

// ============================================================================
// 锁定控制
// ============================================================================

void TargetSelector::force_lock(int target_id)
{
    forced_target_id_ = target_id;
}

void TargetSelector::unlock()
{
    forced_target_id_ = -1;
    // 不清除 current_target_id_，保持当前追踪
}

void TargetSelector::clear_target()
{
    forced_target_id_ = -1;
    current_target_id_ = -1;
    current_armor_idx_ = -1;
    last_seen_time_ = 0;
}

// ============================================================================
// 代价计算
// ============================================================================

std::pair<double, double> TargetSelector::pos_to_yaw_pitch(const Eigen::Vector3d& pos) const
{
    // 假设 pos 是世界坐标系下的位置 (相对于云台)
    // yaw = atan2(y, x)
    // pitch = atan2(-z, √(x² + y²))  (向下为正)
    double yaw = std::atan2(pos.y(), pos.x());
    double pitch = std::atan2(-pos.z(), std::hypot(pos.x(), pos.y()));
    return {yaw, pitch};
}

double TargetSelector::compute_center_distance(
    const Eigen::Vector3d& pos,
    const GimbalState& gimbal) const
{
    // 图像中心 = 当前云台指向
    // 距离 = 目标 yaw/pitch 与当前云台的角度差
    auto [yaw, pitch] = pos_to_yaw_pitch(pos);
    double delta_yaw = GimbalState::normalize_angle(yaw - gimbal.yaw);
    double delta_pitch = pitch - gimbal.pitch;

    // 欧几里得距离 (角度空间)
    return std::hypot(delta_yaw, delta_pitch);
}

double TargetSelector::compute_swing_cost(
    const Eigen::Vector3d& pos,
    const GimbalState& gimbal) const
{
    // 电机转动代价 = √(Δyaw² + Δpitch²)
    // 与 compute_center_distance 相同，但语义不同
    return compute_center_distance(pos, gimbal);
}

// ============================================================================
// 装甲板选择
// ============================================================================

int TargetSelector::select_best_armor(
    const predictor::VehicleState& vehicle,
    const GimbalState& gimbal,
    double dt) const
{
    // 读取参数
    double max_angle = runtime_param::get_param<double>(
        "AutoAim.FireControl.TargetSelector.max_angle"
    );

    int best_idx = -1;
    double min_swing_cost = std::numeric_limits<double>::infinity();

    for (int i = 0; i < vehicle.armor_count; ++i) {
        const auto& armor = vehicle.armors[i];

        // 跳过不可见的 (陀螺模式除外)
        if (!armor.visible && !vehicle.spin.active) continue;

        // 角度过滤 (侧面装甲板不打)
        if (std::abs(armor.z_to_v) > max_angle) continue;

        // 预测位置
        Eigen::Vector3d pos = vehicle.predict_armor_position(i, dt);

        // 计算转动代价
        double cost = compute_swing_cost(pos, gimbal);

        if (cost < min_swing_cost) {
            min_swing_cost = cost;
            best_idx = i;
        }
    }

    return best_idx;
}

bool TargetSelector::should_keep_target(int target_id, double current_time) const
{
    if (target_id != current_target_id_) return false;

    double keep_time = runtime_param::get_param<double>(
        "AutoAim.FireControl.TargetSelector.keep_time"
    );
    return (current_time - last_seen_time_) < keep_time;
}

// ============================================================================
// 旧策略 (注释保留，方便对比)
// ============================================================================

/*
double TargetSelector::compute_projected_area(const predictor::ArmorState& armor) const
{
    double base_area = armor.width() * armor.height();
    double dist_sq = armor.position.squaredNorm();
    if (dist_sq < 0.01) dist_sq = 0.01;
    double cos_angle = std::cos(std::abs(armor.z_to_v));
    return base_area * cos_angle / dist_sq;
}

double TargetSelector::compute_score(
    const predictor::VehicleState& vehicle,
    const predictor::ArmorState& armor,
    double max_area) const
{
    double w_area = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_area");
    double w_still = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_still");
    double w_distance = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_distance");
    double w_priority = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_priority");
    double w_confidence = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_confidence");
    double still_speed_scale = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.still_speed_scale");
    double max_distance = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.max_distance");

    double score = 0;

    // 1. 面积评分
    double area = compute_projected_area(armor);
    double area_score = area / max_area;
    score += w_area * area_score;

    // 2. 静止评分
    if (!vehicle.spin.active || vehicle.spin.level != predictor::SpinLevel::HIGH) {
        double speed = armor.velocity.norm();
        double still_score = 1.0 / (1.0 + speed * still_speed_scale);
        score += w_still * still_score;
    }

    // 3. 距离评分
    double distance = armor.position.norm();
    if (distance > max_distance) {
        return -std::numeric_limits<double>::infinity();
    }
    double dist_score = 1.0 - distance / max_distance;
    score += w_distance * dist_score;

    // 4. 类型优先级
    double type_priority = get_type_priority(vehicle.enemy_type);
    score += w_priority * type_priority;

    // 5. 置信度
    score += w_confidence * vehicle.confidence;

    return score;
}

double TargetSelector::get_type_priority(predictor::EnemyType type) const
{
    using predictor::EnemyType;
    switch (type) {
        case EnemyType::HERO:       return 1.0;
        case EnemyType::INFANTRY_3:
        case EnemyType::INFANTRY_4:
        case EnemyType::INFANTRY_5: return 0.8;
        case EnemyType::SENTRY:     return 0.7;
        case EnemyType::ENGINEER:   return 0.6;
        case EnemyType::OUTPOST:    return 0.5;
        case EnemyType::BASE:       return 0.3;
        default:                    return 0.4;
    }
}

bool TargetSelector::should_switch_target(double new_score, double current_score) const
{
    double switch_hysteresis = runtime_param::get_param<double>(
        "AutoAim.FireControl.TargetSelector.switch_hysteresis"
    );
    return new_score > current_score * (1.0 + switch_hysteresis);
}

bool TargetSelector::should_switch_armor(double new_score, double current_score) const
{
    double switch_armor_hysteresis = runtime_param::get_param<double>(
        "AutoAim.FireControl.TargetSelector.switch_armor_hysteresis"
    );
    return new_score > current_score * (1.0 + switch_armor_hysteresis);
}
*/

}  // namespace autoaim::fire_control
