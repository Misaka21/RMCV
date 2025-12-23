/**
 * @file armor_model.cpp
 * @brief 装甲板运动模型实现 - YPD坐标系 EKF
 */

#include "armor_model.hpp"

#include <cmath>

#include "aimer/common/math/math.hpp"

namespace autoaim::predictor {

// ============================================================================
// FilterThread
// ============================================================================

FilterThread::FilterThread(const ArmorData& armor, double timestamp, double credit_time)
    : armor_(armor), last_update_time_(timestamp), credit_time_(credit_time) {
    // 使用XYZ位置初始化YPD滤波器
    ekf_.init(armor.pos(), timestamp);
}

void FilterThread::update(const ArmorData& armor, double timestamp) {
    // 使用XYZ位置更新 (内部自动转换为YPD)
    ekf_.update(armor.pos(), timestamp);

    // 保存
    armor_ = armor;
    last_update_time_ = timestamp;
}

bool FilterThread::credit(double current_time) const {
    return (current_time - last_update_time_) <= credit_time_;
}

Eigen::Vector3d FilterThread::predict_pos(double timestamp) const {
    return ekf_.predict_pos(timestamp);
}

Eigen::Vector3d FilterThread::predict_vel(double timestamp) const {
    return ekf_.predict_vel(timestamp);
}

math::YpdCoord FilterThread::predict_ypd(double timestamp) const {
    return ekf_.predict_ypd(timestamp);
}

math::YpdCoord FilterThread::predict_ypd_v(double timestamp) const {
    return ekf_.predict_ypd_v(timestamp);
}

ArmorState FilterThread::get_armor_state(double timestamp) const {
    ArmorState as;
    as.id = armor_.id;
    as.type = armor_.type();
    as.z_to_v = armor_.z_to_v();
    as.position = predict_pos(timestamp);
    as.velocity = predict_vel(timestamp);
    as.visible = true;
    as.last_seen = last_update_time_;

    // 评分: z_to_v 越小越好
    as.score = std::max(0.0, 1.0 - std::abs(armor_.z_to_v()));

    return as;
}

// ============================================================================
// ArmorModel
// ============================================================================

ArmorModel::ArmorModel(double credit_time) : credit_time_(credit_time) {}

void ArmorModel::update(const std::vector<ArmorData>& armors, double timestamp) {
    // 更新或创建滤波器
    for (const auto& armor : armors) {
        auto it = filters_.find(armor.id);
        if (it == filters_.end()) {
            // 新建滤波器
            filters_.emplace(armor.id, FilterThread(armor, timestamp, credit_time_));
        } else {
            // 更新现有滤波器
            it->second.update(armor, timestamp);
        }
    }

    // 清理超时滤波器
    for (auto it = filters_.begin(); it != filters_.end();) {
        if (!it->second.credit(timestamp)) {
            it = filters_.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<ArmorState> ArmorModel::get_armor_states(double timestamp) const {
    std::vector<ArmorState> result;
    for (const auto& [id, filter] : filters_) {
        if (filter.credit(timestamp)) {
            result.push_back(filter.get_armor_state(timestamp));
        }
    }
    return result;
}

const FilterThread* ArmorModel::get_best(double timestamp) const {
    const FilterThread* best = nullptr;
    double best_z_to_v = 1e9;

    for (const auto& [id, filter] : filters_) {
        if (!filter.credit(timestamp)) continue;

        double z_to_v = std::abs(filter.armor().z_to_v());
        if (z_to_v < best_z_to_v) {
            best_z_to_v = z_to_v;
            best = &filter;
        }
    }

    return best;
}

void ArmorModel::reset() {
    filters_.clear();
}

}  // namespace autoaim::predictor
