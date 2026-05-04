/**
 * @file enemy_predictor.cpp
 * @brief 敌方预测器实现
 */

#include "enemy_predictor.hpp"

#include <algorithm>
#include <cstdint>
#include <string>

#include <fmt/format.h>

#include "aimer/common/math/math.hpp"
#include "model/enemy_model.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

namespace {

const char* spin_level_name(SpinLevel level)
{
    switch (level) {
        case SpinLevel::LOW: return "LOW";
        case SpinLevel::HIGH: return "HIGH";
        case SpinLevel::NONE:
        default:
            return "NONE";
    }
}

}  // namespace

EnemyPredictor::EnemyPredictor() {
    model_factory_ = std::make_unique<EnemyModelFactory>();
    model_last_seen_time_.fill(-1.0);
    pending_first_seen_time_.fill(-1.0);
    pending_last_seen_time_.fill(-1.0);
}

EnemyPredictor::~EnemyPredictor() = default;

BattlefieldSnapshot EnemyPredictor::predict(const DetectionResult& detection, double timestamp) {
    current_time_ = timestamp;
    frame_id_ = detection.frame_id;
    current_state_ = detection.state;

    // 阶段1: 观测 (PnP)
    update_observations(detection, timestamp);

    // 阶段2: 更新模型 (消抖 + EKF)
    update_models();

    // 导出快照
    return export_snapshot();
}

void EnemyPredictor::update_observations(const DetectionResult& detection, double timestamp) {
    // 告诉 observer 哪些 target_id 有活跃模型
    // 用于 ID 纠正时的"已跟踪"判断，避免用 table 自身结果导致反馈循环。
    // 注意这里会过滤掉“超时但尚未在本帧 update_models() 清理”的旧模型。
    std::set<int> active_ids;
    double stale_limit = runtime_param::get_param<double>("AutoAim.Predictor.lost_timeout");
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (enemy_models_[i] && enemy_models_[i]->alive()
            && model_last_seen_time_[i] >= 0
            && (current_time_ - model_last_seen_time_[i]) <= stale_limit)
        {
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
        model_last_seen_time_[target_id] = current_time_;
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
                model_last_seen_time_[i] = -1.0;
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

    // 从模型导出各目标状态
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (enemy_models_[i] && enemy_models_[i]->alive()) {
            auto target = enemy_models_[i]->predict(current_time_);
            snapshot.add_target(target);

            // 选择置信度最高的作为默认主目标
            // 注意: 实际应由 FireControl 根据代价函数选择
            if (target.confidence > best_confidence) {
                best_confidence = target.confidence;
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

    log_debug_snapshot(snapshot);

    return snapshot;
}

void EnemyPredictor::log_debug_snapshot(const BattlefieldSnapshot& snapshot) const {
    if (!runtime_param::get_param<bool>("AutoAim.Predictor.Debug.enable")) {
        return;
    }

    const int target_id = static_cast<int>(
        runtime_param::get_param<int64_t>("AutoAim.Predictor.Debug.target_id")
    );
    if (target_id <= 0 || target_id >= MAX_TARGETS) {
        return;
    }

    if (last_debug_target_id_ != target_id) {
        last_debug_log_time_ = -1.0;
        last_debug_valid_ = false;
        last_debug_detected_ = false;
        last_debug_spin_level_ = -1;
        last_debug_recommended_idx_ = -2;
        last_debug_visible_mask_ = 0;
        last_debug_obs_count_ = 0;
        last_debug_target_id_ = target_id;
    }

    const auto& observations = observer_.table().get(target_id);
    const size_t obs_count = observations.size();
    const bool detected = snapshot.is_detected(target_id);
    const auto* target = snapshot.find_target(target_id);
    const bool valid = target != nullptr;
    const int spin_level = valid ? static_cast<int>(target->spin.level) : -1;
    const int recommended_idx = valid ? target->recommended_armor_idx : -1;
    const uint8_t visible_mask = valid ? target->visible_mask : 0;

    const double period_s = std::max(
        0.05,
        runtime_param::get_param<double>("AutoAim.Predictor.Debug.period_s")
    );
    const bool periodic = last_debug_log_time_ < 0.0
        || (current_time_ - last_debug_log_time_) >= period_s;
    const bool state_changed = valid != last_debug_valid_
        || detected != last_debug_detected_
        || spin_level != last_debug_spin_level_
        || recommended_idx != last_debug_recommended_idx_
        || visible_mask != last_debug_visible_mask_
        || obs_count != last_debug_obs_count_;

    const bool idle_no_target = !valid && obs_count == 0;
    if (idle_no_target && !state_changed) {
        return;
    }

    if (!periodic && !state_changed) {
        return;
    }

    const bool pending = !valid && obs_count > 0;
    const double pending_age_ms =
        (target_id >= 0 && target_id < MAX_TARGETS && pending_first_seen_time_[target_id] >= 0.0)
            ? (current_time_ - pending_first_seen_time_[target_id]) * 1000.0
            : 0.0;

    if (!valid) {
        debug::print(
            state_changed ? debug::PrintMode::INFO : debug::PrintMode::DEBUG,
            "EnemyPredictor",
            "[T{}-PRED] frame={} obs={} valid=0 detected={} pending={} pending_age={:.0f}ms "
            "primary={} valid_mask=0x{:04x} det_mask=0x{:04x}",
            target_id,
            snapshot.frame_id,
            obs_count,
            detected ? 1 : 0,
            pending ? 1 : 0,
            pending_age_ms,
            snapshot.primary_target_id,
            snapshot.valid_mask,
            snapshot.detected_mask
        );
    } else {
        int visible_count = 0;
        std::string armor_line;
        for (int i = 0; i < target->armor_count; ++i) {
            if (target->armor_visible(i)) {
                ++visible_count;
            }
            const double age_ms = (target->armor_last_seen_time(i) > 0.0)
                ? (current_time_ - target->armor_last_seen_time(i)) * 1000.0
                : -1.0;
            armor_line += fmt::format(
                " i{}(id={},v={},d={},z2v={:+.1f},score={:.2f},age={:.0f}ms,r={:.3f})",
                i,
                target->armor_id(i),
                target->armor_visible(i) ? "Y" : "N",
                target->armor_detected(i) ? "Y" : "N",
                aimer::math::rad2deg(target->predicted_z_to_v(i, 0.0)),
                target->armor_score(i),
                age_ms,
                target->armor_radii[i]
            );
        }

        debug::print(
            state_changed ? debug::PrintMode::INFO : debug::PrintMode::DEBUG,
            "EnemyPredictor",
            "[T{}-PRED] frame={} obs={} valid=1 detected={} primary={} conf={:.2f} "
            "spin={}({}) w={:+.1f}deg/s phase={:+.1f}deg rec={} aid={} vis={}/{} "
            "center=({:.2f},{:.2f},{:.2f}) vel=({:.2f},{:.2f},{:.2f}) "
            "r={:.3f}/{:.3f} dz={:.3f} armors:{}",
            target_id,
            snapshot.frame_id,
            obs_count,
            detected ? 1 : 0,
            snapshot.primary_target_id,
            target->confidence,
            target->spin.active ? 1 : 0,
            spin_level_name(target->spin.level),
            aimer::math::rad2deg(target->spin.omega),
            aimer::math::rad2deg(target->spin.phase),
            target->recommended_armor_idx,
            target->armor_id(target->recommended_armor_idx),
            visible_count,
            target->armor_count,
            target->position.x(),
            target->position.y(),
            target->position.z(),
            target->velocity.x(),
            target->velocity.y(),
            target->velocity.z(),
            target->radius_1,
            target->radius_2,
            target->dz,
            armor_line
        );
    }

    last_debug_log_time_ = current_time_;
    last_debug_valid_ = valid;
    last_debug_detected_ = detected;
    last_debug_spin_level_ = spin_level;
    last_debug_recommended_idx_ = recommended_idx;
    last_debug_visible_mask_ = visible_mask;
    last_debug_obs_count_ = obs_count;
}

void EnemyPredictor::draw(cv::Mat& img, const Eigen::Quaterniond& q_imu, double timestamp) const {
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (enemy_models_[i] && enemy_models_[i]->alive()) {
            enemy_models_[i]->draw(img, q_imu, timestamp);
        }
    }
}

}  // namespace autoaim::predictor
