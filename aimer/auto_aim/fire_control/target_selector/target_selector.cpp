/**
 * @file target_selector.cpp
 * @brief 目标选择器实现
 *
 * 评分: 面积 + 静止 + 距离 + 类型 + 置信度
 * 切换: 迟滞比较，防止震荡
 */

#include "target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoaim::fire_control {

// 装甲板尺寸 (m)
constexpr double SMALL_ARMOR_WIDTH = 0.135;
constexpr double LARGE_ARMOR_WIDTH = 0.230;
constexpr double ARMOR_HEIGHT = 0.055;

TargetSelector::TargetSelector(const Config& config)
    : config_(config)
{}

TargetSelection TargetSelector::select(
    const predictor::BattlefieldSnapshot& snapshot,
    double dt)
{
    TargetSelection best;
    double best_score = -std::numeric_limits<double>::infinity();

    // 第一遍：找最大面积 (用于过滤和归一化)
    double max_area = 0;
    snapshot.for_each_valid([&](int id, const predictor::VehicleState& vehicle) {
        if (!vehicle.valid || vehicle.confidence < 0.1) return;
        for (int i = 0; i < vehicle.armor_count; ++i) {
            const auto& armor = vehicle.armors[i];
            if (!armor.visible && !vehicle.spin.active) continue;
            if (std::abs(armor.z_to_v) > config_.max_angle) continue;

            double area = compute_projected_area(armor);
            max_area = std::max(max_area, area);
        }
    });

    if (max_area < 1e-6) {
        // 没有有效目标
        current_target_id_ = -1;
        current_armor_idx_ = -1;
        current_score_ = 0;
        return best;
    }

    // 如果有强制目标，只考虑该目标
    if (forced_target_id_ >= 0 && snapshot.is_valid(forced_target_id_)) {
        const auto& vehicle = snapshot.vehicles[forced_target_id_];
        int armor_idx = select_best_armor(vehicle, max_area);

        if (armor_idx >= 0) {
            const auto& armor = vehicle.armors[armor_idx];
            best.has_target = true;
            best.target_id = forced_target_id_;
            best.armor_idx = armor_idx;
            best.vehicle = &vehicle;
            best.armor = &armor;
            best.priority = compute_score(vehicle, armor, max_area);
            best.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);

            current_target_id_ = forced_target_id_;
            current_armor_idx_ = armor_idx;
            current_score_ = best.priority;
            return best;
        }
    }

    // 记录当前目标的评分 (用于迟滞比较)
    double current_target_best_score = -std::numeric_limits<double>::infinity();
    TargetSelection current_selection;

    // 第二遍：评分并选择
    snapshot.for_each_valid([&](int id, const predictor::VehicleState& vehicle) {
        if (!vehicle.valid || vehicle.confidence < 0.1) return;

        bool is_current_target = (id == current_target_id_);

        // 选择该目标的最佳装甲板
        int armor_idx = select_best_armor(vehicle, max_area);
        if (armor_idx < 0) return;

        const auto& armor = vehicle.armors[armor_idx];
        double score = compute_score(vehicle, armor, max_area);

        // 记录当前跟踪目标的信息
        if (is_current_target && score > current_target_best_score) {
            current_target_best_score = score;
            current_selection.has_target = true;
            current_selection.target_id = id;
            current_selection.armor_idx = armor_idx;
            current_selection.vehicle = &vehicle;
            current_selection.armor = &armor;
            current_selection.priority = score;
            current_selection.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);
        }

        // 更新全局最佳
        if (score > best_score) {
            best_score = score;
            best.has_target = true;
            best.target_id = id;
            best.armor_idx = armor_idx;
            best.vehicle = &vehicle;
            best.armor = &armor;
            best.priority = score;
            best.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);
        }
    });

    // 迟滞比较：是否切换目标
    if (best.has_target && current_selection.has_target) {
        if (best.target_id != current_target_id_) {
            // 新目标，检查是否应该切换
            if (!should_switch_target(best.priority, current_selection.priority)) {
                // 差距不够大，继续跟踪当前目标
                best = current_selection;
            }
        } else if (best.armor_idx != current_armor_idx_) {
            // 同一目标，不同装甲板
            // 找当前装甲板的评分
            const auto& current_armor = best.vehicle->armors[current_armor_idx_];
            double current_armor_score = compute_score(*best.vehicle, current_armor, max_area);

            if (!should_switch_armor(best.priority, current_armor_score)) {
                // 差距不够大，继续跟踪当前装甲板
                best.armor_idx = current_armor_idx_;
                best.armor = &current_armor;
                best.priority = current_armor_score;
                best.predicted_pos = best.vehicle->predict_armor_position(current_armor_idx_, dt);
            }
        }
    }

    // 更新状态
    if (best.has_target) {
        current_target_id_ = best.target_id;
        current_armor_idx_ = best.armor_idx;
        current_score_ = best.priority;
    } else {
        current_target_id_ = -1;
        current_armor_idx_ = -1;
        current_score_ = 0;
    }

    return best;
}

void TargetSelector::force_target(int target_id)
{
    forced_target_id_ = target_id;
}

void TargetSelector::clear_target()
{
    forced_target_id_ = -1;
    current_target_id_ = -1;
    current_armor_idx_ = -1;
    current_score_ = 0;
}

double TargetSelector::compute_projected_area(const predictor::ArmorState& armor) const
{
    // 装甲板实际尺寸
    double width = (armor.type == predictor::ArmorType::LARGE)
        ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
    double base_area = width * ARMOR_HEIGHT;

    // 距离平方 (投影面积与距离平方成反比)
    double dist_sq = armor.position.squaredNorm();
    if (dist_sq < 0.01) dist_sq = 0.01;

    // 朝向修正 (正对时最大，侧面时减小)
    double cos_angle = std::cos(std::abs(armor.z_to_v));

    // 投影面积
    return base_area * cos_angle / dist_sq;
}

double TargetSelector::compute_score(
    const predictor::VehicleState& vehicle,
    const predictor::ArmorState& armor,
    double max_area) const
{
    double score = 0;

    // 1. 面积评分 (0~1, 归一化到最大面积)
    double area = compute_projected_area(armor);
    double area_score = area / max_area;
    score += config_.w_area * area_score;

    // 2. 静止评分 (0~1, 速度越小越好)
    // 注意: 高速陀螺的装甲板切向速度大但可预测，不适用此评分
    // 火控层面会根据 spin.level 决定打 armor 还是 center
    if (!vehicle.spin.active || vehicle.spin.level != predictor::SpinLevel::HIGH) {
        double speed = armor.velocity.norm();
        double still_score = 1.0 / (1.0 + speed * config_.still_speed_scale);
        score += config_.w_still * still_score;
    }

    // 3. 距离评分 (0~1, 近优先)
    double distance = armor.position.norm();
    if (distance > config_.max_distance) {
        return -std::numeric_limits<double>::infinity();
    }
    double dist_score = 1.0 - distance / config_.max_distance;
    score += config_.w_distance * dist_score;

    // 4. 类型优先级
    double type_priority = get_type_priority(vehicle.enemy_type);
    score += config_.w_priority * type_priority;

    // 5. 置信度
    score += config_.w_confidence * vehicle.confidence;

    return score;
}

double TargetSelector::get_type_priority(predictor::EnemyType type) const
{
    using predictor::EnemyType;

    switch (type) {
        case EnemyType::HERO:       return 1.0;   // 英雄最高
        case EnemyType::INFANTRY_3:
        case EnemyType::INFANTRY_4:
        case EnemyType::INFANTRY_5: return 0.8;   // 步兵
        case EnemyType::SENTRY:     return 0.7;   // 哨兵
        case EnemyType::ENGINEER:   return 0.6;   // 工程
        case EnemyType::OUTPOST:    return 0.5;   // 前哨站
        case EnemyType::BASE:       return 0.3;   // 基地
        default:                    return 0.4;
    }
}

int TargetSelector::select_best_armor(
    const predictor::VehicleState& vehicle,
    double max_area) const
{
    int best_idx = -1;
    double best_score = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < vehicle.armor_count; ++i) {
        const auto& armor = vehicle.armors[i];

        // 跳过不可见的 (陀螺模式除外)
        if (!armor.visible && !vehicle.spin.active) continue;

        // 角度过滤
        if (std::abs(armor.z_to_v) > config_.max_angle) continue;

        // 面积过滤
        double area = compute_projected_area(armor);
        if (area < max_area * config_.area_filter_ratio) continue;

        // 简单评分: 面积 + 静止
        double speed = armor.velocity.norm();
        double score = area / max_area + 1.0 / (1.0 + speed * config_.still_speed_scale);

        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    return best_idx;
}

bool TargetSelector::should_switch_target(double new_score, double current_score) const
{
    // 新目标必须比当前目标好 hysteresis 比例才切换
    return new_score > current_score * (1.0 + config_.switch_hysteresis);
}

bool TargetSelector::should_switch_armor(double new_score, double current_score) const
{
    // 同一目标内切换装甲板，阈值可以小一点
    return new_score > current_score * (1.0 + config_.switch_armor_hysteresis);
}

}  // namespace autoaim::fire_control
