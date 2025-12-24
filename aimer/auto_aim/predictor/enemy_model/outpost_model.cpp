/**
 * @file outpost_model.cpp
 * @brief 前哨站运动模型实现 (使用 EKF)
 */

#include "outpost_model.hpp"

#include <cmath>

namespace autoaim::predictor {

OutpostModel::OutpostModel(int target_id, EnemyType enemy_type)
    : target_id_(target_id)
    , enemy_type_(enemy_type) {}

void OutpostModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    ++frame_count_;

    // 盲区处理: 没有观测也更新时间戳 (用于 alive() 判断)
    // 但不 reset，让 motion_ 继续预测
    if (observations.empty()) {
        if (initialized_ && (timestamp - last_update_time_) > LOST_TIMEOUT) {
            reset();
        }
        return;
    }

    last_update_time_ = timestamp;

    // ArmorIdentifier 分配 ID
    identifier_.update(observations, timestamp, frame_count_);
    auto armors_with_id = identifier_.get_active_armors(frame_count_);
    if (armors_with_id.empty()) return;

    // 选择最正对的装甲板
    const ArmorData* best = nullptr;
    double best_z_to_v = 1e9;
    for (const auto& armor : armors_with_id) {
        if (armor.z_to_v() < best_z_to_v) {
            best_z_to_v = armor.z_to_v();
            best = &armor;
        }
    }

    if (!best) return;

    // 更新 EKF 运动模型
    motion_.update(*best, timestamp);
    initialized_ = motion_.valid();
}

VehicleState OutpostModel::predict(double timestamp) const {
    VehicleState vs;
    vs.target_id = target_id_;
    vs.enemy_type = enemy_type_;
    vs.valid = initialized_;

    if (!initialized_) return vs;

    // 计算预测时间差
    double dt = timestamp - last_update_time_;

    // 从 EKF 模型获取预测
    Eigen::Vector3d center = motion_.predict_center(dt);
    Eigen::Vector3d velocity = motion_.get_velocity();
    double omega = motion_.get_omega();
    double theta = motion_.get_theta() + omega * dt;

    vs.center = center;
    vs.velocity = velocity;
    vs.timestamp = timestamp;
    vs.armor_count = ARMOR_NUM;

    // 陀螺状态
    vs.spin.active = true;
    vs.spin.omega = omega;
    vs.spin.phase = theta;
    vs.spin.radius = outpost::RADIUS;
    vs.spin.level = SpinLevel::HIGH;  // 前哨站始终高速

    // 生成 3 块装甲板位置
    for (int i = 0; i < ARMOR_NUM; ++i) {
        Eigen::Vector3d armor_pos = motion_.predict_armor_pos(i, dt);

        vs.armors[i].id = i;
        vs.armors[i].type = ArmorType::SMALL;
        vs.armors[i].position = armor_pos;
        vs.armors[i].velocity = velocity;  // 继承中心速度
        vs.armors[i].visible = true;

        // 装甲板朝向 = 中心到装甲板的方向
        double armor_theta = theta + i * (2.0 * M_PI / 3.0);
        vs.armors[i].yaw = armor_theta;

        // 评分: 当前槽位得分最高
        vs.armors[i].score = (i == motion_.get_current_slot()) ? 1.0 : 0.5;
    }

    // 推荐装甲板 = 当前追踪的槽位
    vs.recommended_armor_idx = motion_.get_current_slot();

    return vs;
}

bool OutpostModel::alive() const {
    return initialized_;
}

void OutpostModel::reset() {
    initialized_ = false;
    motion_.reset();
    identifier_.reset();
    frame_count_ = 0;
}

}  // namespace autoaim::predictor
