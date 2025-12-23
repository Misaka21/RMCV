/**
 * @file vehicle_model.cpp
 * @brief 车辆运动模型实现
 */

#include "vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include "aimer/common/math/math.hpp"

namespace autoaim::predictor {

VehicleModel::VehicleModel(int target_id, EnemyType enemy_type)
    : target_id_(target_id),
      enemy_type_(enemy_type),
      armor_model_(ARMOR_CREDIT_TIME) {}

void VehicleModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    ++frame_count_;

    // 1. 消抖过滤
    auto filtered = filter(observations, prev_armors_);

    if (filtered.empty()) {
        if (initialized_ && (timestamp - last_update_time_) > LOST_TIMEOUT) {
            reset();
        }
        return;
    }

    last_update_time_ = timestamp;

    // 2. ArmorIdentifier: ID 分配
    identifier_.update(filtered, timestamp, frame_count_);

    // 3. ArmorModel: EKF 滤波
    auto armors_with_id = identifier_.get_active_armors(frame_count_);
    armor_model_.update(armors_with_id, timestamp);

    if (!initialized_) {
        initialized_ = true;
    }

    // 4. 更新陀螺状态 (TODO)
    // spin_.update_level(ekf_.x()[OMEGA]);

    prev_armors_ = filtered;
    prev_timestamp_ = timestamp;
}

std::vector<ArmorObservation> VehicleModel::filter(
    const std::vector<ArmorObservation>& raw,
    const std::vector<ArmorObservation>& last
) const {
    std::vector<ArmorObservation> result;

    if (raw.empty()) return result;

    double max_area = 0;
    for (const auto& a : raw) {
        if (!a.valid) continue;
        double area = math::get_area(a.pts);
        if (area > max_area) max_area = area;
    }

    std::set<int> last_ids;
    for (const auto& a : last) {
        last_ids.insert(a.armor_id);
    }

    double dt = last_update_time_ - prev_timestamp_;
    bool do_jump_check = !last.empty() && dt > 0 && dt < LOST_TIMEOUT;

    for (const auto& a : raw) {
        if (!a.valid) continue;

        double area = math::get_area(a.pts);

        bool is_existing = last_ids.count(a.armor_id) > 0;
        double area_thresh = is_existing
            ? max_area * EXISTING_ARMOR_AREA_RATIO
            : max_area * NEW_ARMOR_AREA_RATIO;
        if (area < area_thresh) continue;

        double dist = a.distance();
        if (dist < MIN_DIST || dist > MAX_DIST) continue;
        if (!is_existing && dist > NEW_ARMOR_MAX_DIST) continue;
        if (std::abs(a.z_to_v) > MAX_Z_TO_V) continue;

        if (do_jump_check) {
            double min_jump = 1e9;
            for (const auto& o : last) {
                double d = (a.pos - o.pos).norm();
                min_jump = std::min(min_jump, d);
            }
            if (min_jump > JUMP_DISTANCE_LIMIT) continue;
        }

        result.push_back(a);
    }

    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        return a.z_to_v < b.z_to_v;
    });

    if (result.size() > 4) {
        result.resize(4);
    }

    return result;
}

VehicleState VehicleModel::predict(double timestamp) const {
    VehicleState vs;
    vs.target_id = target_id_;
    vs.enemy_type = enemy_type_;
    vs.valid = initialized_;
    vs.timestamp = timestamp;
    vs.frame_count = frame_count_;

    if (!initialized_) return vs;

    // 从 ArmorModel 获取滤波后的装甲板状态
    auto armor_states = armor_model_.get_armor_states(timestamp);

    vs.armor_count = static_cast<int>(std::min(armor_states.size(), size_t(MAX_ARMORS_PER_TARGET)));
    vs.spin = spin_;

    double best_score = -1;
    int best_idx = -1;
    Eigen::Vector3d center_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_sum = Eigen::Vector3d::Zero();
    int valid_count = 0;

    for (int i = 0; i < vs.armor_count; ++i) {
        vs.armors[i] = armor_states[i];

        center_sum += armor_states[i].position;
        vel_sum += armor_states[i].velocity;
        ++valid_count;

        if (armor_states[i].score > best_score) {
            best_score = armor_states[i].score;
            best_idx = i;
        }
    }

    if (valid_count > 0) {
        vs.center = center_sum / valid_count;
        vs.velocity = vel_sum / valid_count;
    }

    vs.recommended_armor_idx = best_idx;

    return vs;
}

bool VehicleModel::alive() const {
    return initialized_;
}

void VehicleModel::reset() {
    initialized_ = false;
    identifier_.reset();
    armor_model_.reset();
    prev_armors_.clear();
    spin_.reset();
    frame_count_ = 0;
}

}  // namespace autoaim::predictor
