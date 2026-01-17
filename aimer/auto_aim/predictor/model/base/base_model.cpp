/**
 * @file base_model.cpp
 * @brief 基地运动模型实现
 */

#include "base_model.hpp"

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
    double best_z_to_v = 1e9;
    for (const auto& obs : observations) {
        if (obs.valid && obs.z_to_v < best_z_to_v) {
            best_z_to_v = obs.z_to_v;
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

VehicleState BaseModel::predict(double timestamp) const {
    VehicleState vs;
    vs.target_id = target_id_;
    vs.enemy_type = enemy_type_;
    vs.valid = initialized_;

    if (!initialized_) return vs;

    vs.center = position_;
    vs.velocity = Eigen::Vector3d::Zero();
    vs.timestamp = timestamp;
    vs.armor_count = 1;

    // 单个装甲板
    vs.armors[0].id = 0;
    vs.armors[0].position = position_;
    vs.armors[0].velocity = Eigen::Vector3d::Zero();
    vs.armors[0].visible = true;

    return vs;
}

bool BaseModel::alive() const {
    return initialized_;
}

void BaseModel::reset() {
    initialized_ = false;
    position_ = Eigen::Vector3d::Zero();
}

}  // namespace autoaim::predictor
