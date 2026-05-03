/**
 * @file base_model.cpp
 * @brief 基地运动模型实现
 */

#include "base_model.hpp"

#include <cmath>

namespace autoaim::predictor {

BaseModel::BaseModel(int target_id, EnemyType enemy_type)
    : target_id_(target_id), enemy_type_(enemy_type) {}

void BaseModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    if (observations.empty()) {
        if (initialized_ && (timestamp - last_update_time_) > LOST_TIMEOUT) {
            reset();
        }
        return;
    }

    last_update_time_ = timestamp;

    // 选择最佳观测
    const ArmorObservation* best = nullptr;
    double best_abs_z_to_v = 1e9;
    for (const auto& obs : observations) {
        double abs_z_to_v = std::abs(obs.z_to_v);
        if (obs.valid && abs_z_to_v < best_abs_z_to_v) {
            best_abs_z_to_v = abs_z_to_v;
            best = &obs;
        }
    }

    if (!best) return;

    if (!initialized_) {
        position_ = best->pos;
        initialized_ = true;
        return;
    }

    // 简单低通滤波
    position_ = (1.0 - FILTER_ALPHA) * position_ + FILTER_ALPHA * best->pos;
}

TargetState BaseModel::predict(double timestamp) const {
    TargetState target;
    target.target_id = target_id_;
    target.enemy_type = enemy_type_;
    target.valid = initialized_;
    target.tracking = initialized_;

    if (!initialized_) return target;

    target.position = position_;
    target.velocity = Eigen::Vector3d::Zero();
    target.timestamp = timestamp;
    target.armor_count = 1;

    // 单个装甲板
    target.armor_ids[0] = 0;
    target.armor_types[0] = ArmorType::SMALL;
    target.armor_position_offsets[0] = Eigen::Vector3d::Zero();
    target.armor_velocity_offsets[0] = Eigen::Vector3d::Zero();
    target.armor_z_to_v[0] = 0.0;
    target.armor_last_seen[0] = last_update_time_;
    target.armor_scores[0] = 1.0;
    target.recommended_armor_idx = 0;
    target.set_armor_visible(0, true);
    target.set_armor_detected(0, true);

    return target;
}

bool BaseModel::alive() const {
    return initialized_;
}

void BaseModel::reset() {
    initialized_ = false;
    position_ = Eigen::Vector3d::Zero();
}

}  // namespace autoaim::predictor
