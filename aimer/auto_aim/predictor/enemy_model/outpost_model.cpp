/**
 * @file outpost_model.cpp
 * @brief 前哨站运动模型实现
 */

#include "outpost_model.hpp"

#include <cmath>

namespace autoaim::predictor {

OutpostModel::OutpostModel(int target_id, EnemyType enemy_type)
    : target_id_(target_id), enemy_type_(enemy_type) {}

void OutpostModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    if (observations.empty()) {
        if (initialized_ && (timestamp - last_update_time_) > LOST_TIMEOUT) {
            reset();
        }
        return;
    }

    double dt = timestamp - last_update_time_;
    last_update_time_ = timestamp;

    // 选择最佳观测 (最正对的)
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
        // 首次初始化
        center_ = best->pos;
        phase_ = std::atan2(best->pos.y() - center_.y(), best->pos.x() - center_.x());
        initialized_ = true;
        return;
    }

    // 更新相位
    if (dt > 0 && dt < 0.5) {
        phase_ += omega_ * dt;
    }

    // 用观测修正
    Eigen::Vector3d measured_center = best->pos;
    center_ = 0.9 * center_ + 0.1 * measured_center;

    double measured_phase = std::atan2(
        best->pos.y() - center_.y(),
        best->pos.x() - center_.x()
    );

    // 估计角速度
    if (dt > 0) {
        double dphase = measured_phase - phase_;
        while (dphase > M_PI) dphase -= 2 * M_PI;
        while (dphase < -M_PI) dphase += 2 * M_PI;
        omega_ = 0.8 * omega_ + 0.2 * (dphase / dt);
    }

    phase_ = measured_phase;
}

VehicleState OutpostModel::predict(double timestamp) const {
    VehicleState vs;
    vs.target_id = target_id_;
    vs.enemy_type = enemy_type_;
    vs.valid = initialized_;

    if (!initialized_) return vs;

    double dt = timestamp - last_update_time_;
    double pred_phase = phase_ + omega_ * dt;

    vs.center = center_;
    vs.velocity = Eigen::Vector3d::Zero();
    vs.timestamp = timestamp;
    vs.armor_count = ARMOR_NUM;

    // 陀螺状态
    vs.spin.active = true;
    vs.spin.omega = omega_;
    vs.spin.phase = pred_phase;
    vs.spin.radius = radius_;

    // 生成 3 块装甲板
    for (int i = 0; i < ARMOR_NUM; ++i) {
        double angle = pred_phase + i * ARMOR_ANGLE_STEP;
        vs.armors[i].id = i;
        vs.armors[i].type = ArmorType::SMALL;
        vs.armors[i].position = Eigen::Vector3d(
            center_.x() + radius_ * std::cos(angle),
            center_.y() + radius_ * std::sin(angle),
            center_.z()
        );
        vs.armors[i].yaw = angle;
    }

    return vs;
}

bool OutpostModel::alive() const {
    return initialized_;
}

void OutpostModel::reset() {
    initialized_ = false;
    center_ = Eigen::Vector3d::Zero();
    omega_ = 0;
    phase_ = 0;
}

}  // namespace autoaim::predictor
