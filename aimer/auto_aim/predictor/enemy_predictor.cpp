/**
 * @file enemy_predictor.cpp
 * @brief 敌方预测器实现
 */

#include "enemy_predictor.hpp"

#include "enemy_model/enemy_model.hpp"

namespace autoaim::predictor {

EnemyPredictor::EnemyPredictor() {
    model_factory_ = std::make_unique<EnemyModelFactory>();
}

EnemyPredictor::~EnemyPredictor() = default;

void EnemyPredictor::set_camera_params(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    observer_.set_camera_params(camera_matrix, dist_coeffs);
}

BattlefieldSnapshot EnemyPredictor::predict(const autoaim::DetectionResult& detection) {
    current_time_ = detection.timestamp;
    frame_id_++;

    // 阶段1: 观测 (PnP)
    update_observations(detection);

    // 阶段2: 更新模型 (消抖 + EKF)
    update_models();

    // 选择目标
    select_target();

    // 导出快照
    return export_snapshot();
}

void EnemyPredictor::update_observations(const autoaim::DetectionResult& detection) {
    observer_.observe(detection, detection.q_imu);
}

void EnemyPredictor::update_models() {
    const auto& table = observer_.table();

    // 遍历所有检测到的目标
    for (int target_id : table.get_target_ids()) {
        if (target_id <= 0 || target_id >= MAX_TARGETS) continue;

        const auto& observations = table.get(target_id);
        if (observations.empty()) continue;

        // 确定目标类型
        EnemyType enemy_type = static_cast<EnemyType>(target_id);
        if (!observations.empty()) {
            // 从观测中获取类型 (ArmorNumber)
            enemy_type = static_cast<EnemyType>(observations[0].target_id);
        }

        // 如果模型不存在，创建
        if (!enemy_models_[target_id]) {
            enemy_models_[target_id] = model_factory_->create(target_id, enemy_type);
        }

        // 更新模型
        enemy_models_[target_id]->update(observations, current_time_);
    }

    // 更新未检测到的目标 (传入空观测，让模型判断超时)
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (enemy_models_[i] && !table.has(i)) {
            static const std::vector<ArmorObservation> empty;
            enemy_models_[i]->update(empty, current_time_);

            // 如果模型失效，销毁
            if (!enemy_models_[i]->alive()) {
                enemy_models_[i].reset();
            }
        }
    }
}

void EnemyPredictor::select_target() {
    const auto& table = observer_.table();

    // 尝试锁定最近的目标
    double min_dist = 1e9;
    int best_id = -1;
    EnemyType best_type = EnemyType::UNKNOWN;

    for (int target_id : table.get_target_ids()) {
        const auto& obs = table.get(target_id);
        if (obs.empty()) continue;

        // 找最近的观测
        for (const auto& a : obs) {
            double dist = a.distance();
            if (dist < min_dist) {
                min_dist = dist;
                best_id = target_id;
                best_type = static_cast<EnemyType>(a.target_id);
            }
        }
    }

    if (best_id > 0) {
        target_catcher_.try_catch(best_id, best_type, current_time_);
    }
}

BattlefieldSnapshot EnemyPredictor::export_snapshot() {
    BattlefieldSnapshot snapshot;
    snapshot.timestamp = current_time_;
    snapshot.frame_id = frame_id_;
    snapshot.clear();

    const auto& table = observer_.table();

    // 从模型导出各车辆状态
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (enemy_models_[i] && enemy_models_[i]->alive()) {
            snapshot.vehicles[i] = enemy_models_[i]->predict(current_time_);
            snapshot.set_valid(i, true);
        }

        // 标记当前帧检测到的
        if (table.has(i)) {
            snapshot.set_detected(i, true);
        }
    }

    // 设置主目标
    snapshot.primary_target_id = target_catcher_.get_target(current_time_);

    return snapshot;
}

}  // namespace autoaim::predictor
