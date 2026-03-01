/**
 * @file enemy_predictor.cpp
 * @brief 敌方预测器实现
 */

#include "enemy_predictor.hpp"

#include "model/enemy_model.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

EnemyPredictor::EnemyPredictor() {
    model_factory_ = std::make_unique<EnemyModelFactory>();
    pending_first_seen_time_.fill(-1.0);
    pending_last_seen_time_.fill(-1.0);
}

EnemyPredictor::~EnemyPredictor() = default;

BattlefieldSnapshot EnemyPredictor::predict(const DetectionResult& detection, double timestamp) {
    current_time_ = timestamp;
    frame_id_ = detection.frame_id;  // 使用相机原始帧号
    current_state_ = detection.state;  // 保存自身状态

    // 阶段1: 观测 (PnP)
    update_observations(detection, timestamp);

    // 阶段2: 更新模型 (消抖 + EKF)
    update_models();

    // 导出快照
    return export_snapshot();
}

void EnemyPredictor::update_observations(const DetectionResult& detection, double timestamp) {
    // 告诉 observer 哪些 target_id 有活跃模型
    // 用于 ID 纠正时的"已跟踪"判断，避免用 table 自身结果导致反馈循环
    std::set<int> active_ids;
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (enemy_models_[i] && enemy_models_[i]->alive()) {
            active_ids.insert(i);
        }
    }
    observer_.set_active_model_ids(active_ids);
    observer_.observe(detection, timestamp);
}

void EnemyPredictor::update_models() {
    const auto& table = observer_.table();

    // 新目标消抖参数: 防止 detector number 抖动导致一辆车变多辆
    double confirm_time = runtime_param::get_param<double>(
        "AutoAim.Predictor.IDDebounce.new_target_confirm_time"
    );
    double max_gap = runtime_param::get_param<double>(
        "AutoAim.Predictor.IDDebounce.new_target_max_gap"
    );
    double pending_timeout = runtime_param::get_param<double>(
        "AutoAim.Predictor.IDDebounce.pending_timeout"
    );

    std::array<bool, MAX_TARGETS> seen_this_frame = {};

    // 遍历所有检测到的目标
    for (int target_id : table.get_target_ids()) {
        if (target_id <= 0 || target_id >= MAX_TARGETS) continue;
        seen_this_frame[target_id] = true;

        const auto& observations = table.get(target_id);
        if (observations.empty()) continue;

        if (!enemy_models_[target_id]) {
            // 该 target_id 尚未建模，先做连续出现确认
            bool continuous = pending_first_seen_time_[target_id] >= 0
                && pending_last_seen_time_[target_id] >= 0
                && (current_time_ - pending_last_seen_time_[target_id]) <= max_gap;

            if (!continuous) {
                pending_first_seen_time_[target_id] = current_time_;
            }

            pending_last_seen_time_[target_id] = current_time_;

            double seen_duration = current_time_ - pending_first_seen_time_[target_id];
            if (seen_duration < confirm_time) {
                continue;  // 未达到确认时间，不创建新模型
            }

            // 从观测中获取目标类型并创建模型
            EnemyType enemy_type = static_cast<EnemyType>(observations[0].target_id);
            enemy_models_[target_id] = model_factory_->create(target_id, enemy_type);

            // 清空 pending 状态
            pending_first_seen_time_[target_id] = -1.0;
            pending_last_seen_time_[target_id] = -1.0;
        }

        // 更新模型
        enemy_models_[target_id]->update(observations, current_time_);
    }

    // 清理长时间未续上的 pending 候选
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (seen_this_frame[i]) continue;
        if (pending_first_seen_time_[i] < 0) continue;
        if ((current_time_ - pending_last_seen_time_[i]) > pending_timeout) {
            pending_first_seen_time_[i] = -1.0;
            pending_last_seen_time_[i] = -1.0;
        }
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

BattlefieldSnapshot EnemyPredictor::export_snapshot() {
    BattlefieldSnapshot snapshot;
    snapshot.clear();
    snapshot.timestamp = current_time_;
    snapshot.frame_id = frame_id_;
    snapshot.self_state = current_state_;

    const auto& table = observer_.table();

    // 用于选择主目标
    int best_target_id = -1;
    double best_confidence = -1;

    // 从模型导出各车辆状态
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (enemy_models_[i] && enemy_models_[i]->alive()) {
            snapshot.vehicles[i] = enemy_models_[i]->predict(current_time_);
            snapshot.set_valid(i, true);

            // 选择置信度最高的作为默认主目标
            // 注意: 实际应由 FireControl 根据代价函数选择
            if (snapshot.vehicles[i].confidence > best_confidence) {
                best_confidence = snapshot.vehicles[i].confidence;
                best_target_id = i;
            }
        }

        // 标记当前帧检测到的
        if (table.has(i)) {
            snapshot.set_detected(i, true);
        }
    }

    // 设置默认主目标 (FireControl 可以覆盖)
    snapshot.primary_target_id = best_target_id;

    return snapshot;
}

void EnemyPredictor::draw(cv::Mat& img, const Eigen::Quaterniond& q_imu, double timestamp) const {
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (enemy_models_[i] && enemy_models_[i]->alive()) {
            enemy_models_[i]->draw(img, q_imu, timestamp);
        }
    }
}

}  // namespace autoaim::predictor
