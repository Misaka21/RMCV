/**
 * @file target_selector.cpp
 * @brief 目标选择器实现
 *
 * 职责: 只选"打哪个敌人"，不选装甲板
 *       装甲板选择由 ArmorAim 负责
 *
 * 选择策略 (参考 rm.cv.fans):
 *   选敌人：最靠近图像中心 (操作手意图)
 *
 * 锁定机制:
 *   - 当有目标且可见时，保持当前目标
 *   - top1/top2 允许“无可见板继续跟踪”（供 indirect）
 *   - 当目标丢失超过 keep_time 后才切换
 */

#include "target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

namespace {

double get_param_or(const std::string& name, double default_value)
{
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (auto* val = std::get_if<double>(&*ptr)) {
            return *val;
        }
    }
    return default_value;
}

int pick_fallback_armor_idx(const predictor::TargetState& target) {
    if (target.armor_count <= 0) return -1;

    if (target.armor_index_valid(target.recommended_armor_idx))
    {
        return target.recommended_armor_idx;
    }

    for (int i = 0; i < target.armor_count; ++i) {
        if (target.armor_visible(i)) return i;
    }

    const int best_idx = target.best_armor_idx();
    if (target.armor_index_valid(best_idx)) {
        return best_idx;
    }

    return target.armor_index_valid(0) ? 0 : -1;
}

}  // namespace

// ============================================================================
// 主选择函数（参考 rm.cv.fans TargetCatcher：抓取-保持-超时释放）
// ============================================================================

TargetSelection TargetSelector::select(
    const predictor::BattlefieldSnapshot& snapshot,
    const GimbalState& gimbal,
    double dt)
{
    TargetSelection result;
    const double current_time = snapshot.timestamp;

    // ========== 1. 预瞄锁定优先 ==========
    if (forced_target_id_ >= 0) {
        const auto* target = snapshot.find_target(forced_target_id_);
        if (target != nullptr && is_trackable_target(*target, dt, current_time)) {
            const auto forced_candidate = evaluate_visible_candidate(
                forced_target_id_, *target, gimbal, dt
            );
            int armor_idx = forced_candidate.valid()
                ? forced_candidate.armor_idx
                : pick_fallback_armor_idx(*target);
            if (armor_idx >= 0) {
                current_target_id_ = forced_target_id_;
                target_caught_time_ = current_time;

                result.has_target = true;
                result.target_id = forced_target_id_;
                result.priority = forced_candidate.valid()
                    ? (1.0 / (1.0 + forced_candidate.score))
                    : 0.0;
                result.predicted_pos = target->predict_armor_position(armor_idx, dt);
                return result;
            }
        }
    }

    // ========== 2. 构造”最中心可见目标”候选 ==========
    // 对齐 rmcvfans: 候选池只看当前帧检测到的目标 (sorted_armors 只来自当帧)
    VisibleCandidate best_candidate;
    snapshot.for_each_valid([&](int id, const predictor::TargetState& vehicle) {
        if (!snapshot.is_detected(id)) return;
        auto candidate = evaluate_visible_candidate(id, vehicle, gimbal, dt);
        if (!candidate.valid()) return;
        if (candidate.score < best_candidate.score) {
            best_candidate = candidate;
        }
    });

    // ========== 3. 按 TargetCatcher 逻辑更新锁存目标 ==========
    // 对齐 rmcvfans: 刷新条件用 is_detected (当帧有数据)，而非 is_trackable_target。
    // credit bridge 只在 Step 4 输出层使用，不影响选车切换。
    const int tracked_target = latched_target(current_time);
    if (tracked_target < 0) {
        current_target_id_ = -1;
    }
    if (tracked_target >= 0 && snapshot.is_valid(tracked_target)) {
        if (snapshot.is_detected(tracked_target)) {
            target_caught_time_ = current_time;
        } else if (best_candidate.valid()) {
            try_catch_target(
                best_candidate, snapshot, current_time
            );
        }
    } else if (best_candidate.valid()) {
        try_catch_target(
            best_candidate, snapshot, current_time
        );
    }

    // ========== 4. 输出当前锁存目标 ==========
    const int selected_target = latched_target(current_time);
    if (selected_target < 0 || !snapshot.is_valid(selected_target)) {
        return result;
    }

    const auto* selected_state = snapshot.find_target(selected_target);
    if (selected_state == nullptr || !is_trackable_target(*selected_state, dt, current_time)) {
        return result;
    }

    const auto selected_visible = evaluate_visible_candidate(
        selected_target, *selected_state, gimbal, dt
    );
    int armor_idx = selected_visible.valid()
        ? selected_visible.armor_idx
        : pick_fallback_armor_idx(*selected_state);
    if (armor_idx < 0) {
        return result;
    }

    result.has_target = true;
    result.target_id = selected_target;
    result.priority = selected_visible.valid()
        ? (1.0 / (1.0 + selected_visible.score))
        : 0.0;
    result.predicted_pos = selected_state->predict_armor_position(armor_idx, dt);
    return result;
}

// ============================================================================
// 锁定控制
// ============================================================================

void TargetSelector::force_lock(int target_id)
{
    forced_target_id_ = target_id;
}

void TargetSelector::unlock()
{
    forced_target_id_ = -1;
    // 不清除 current_target_id_，保持当前追踪
}

void TargetSelector::clear_target()
{
    forced_target_id_ = -1;
    current_target_id_ = -1;
    target_caught_time_ = 0;
}

// ============================================================================
// 辅助方法
// ============================================================================

std::pair<double, double> TargetSelector::pos_to_yaw_pitch(const Eigen::Vector3d& pos) const
{
    // 假设 pos 是世界坐标系下的位置 (相对于云台)
    // yaw = atan2(y, x)
    // pitch = atan2(z, √(x² + y²))  (与项目统一约定: z 上为正, pitch 上为正)
    double yaw = std::atan2(pos.y(), pos.x());
    double pitch = std::atan2(pos.z(), std::hypot(pos.x(), pos.y()));
    return {yaw, pitch};
}

double TargetSelector::compute_center_distance(
    const Eigen::Vector3d& pos,
    const GimbalState& gimbal) const
{
    // 图像中心 = 当前云台指向
    // 距离 = 目标 yaw/pitch 与当前云台的角度差
    auto [yaw, pitch] = pos_to_yaw_pitch(pos);
    double delta_yaw = GimbalState::normalize_angle(yaw - gimbal.yaw);
    double delta_pitch = pitch - gimbal.pitch;

    // 欧几里得距离 (角度空间)
    return std::hypot(delta_yaw, delta_pitch);
}

bool TargetSelector::has_visible_armor(
    const predictor::TargetState& target,
    double /* dt */
) const
{
    for (int i = 0; i < target.armor_count; ++i) {
        // 仅要求“有可见板”即可参与选敌，避免窗口硬过滤导致无目标。
        if (target.armor_visible(i)) return true;
    }
    return false;
}

bool TargetSelector::can_track_without_visible(
    const predictor::TargetState& target,
    double current_time
) const
{
    if (!target.spin.active) {
        return false;
    }
    if (target.spin.level == predictor::SpinLevel::NONE) {
        return false;
    }

    // 对齐 rm.cv.fans: allow_indirect 由 top_level 决定，而不是由 orientation window 是否大于 0 决定。
    // 这里用“最近一次有效观测时间”近似 top credit，避免在两板空窗期直接丢目标。
    double credit_time = get_param_or(
        "AutoAim.FireControl.TargetSelector.top_credit_time", 1.0
    );
    if (target.enemy_type == predictor::EnemyType::OUTPOST) {
        credit_time = get_param_or(
            "AutoAim.FireControl.TargetSelector.top_credit_time_outpost", credit_time
        );
    }
    credit_time = std::max(0.0, credit_time);

    double latest_seen = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < target.armor_count; ++i) {
        latest_seen = std::max(latest_seen, target.armor_last_seen_time(i));
    }
    if (!std::isfinite(latest_seen) || latest_seen <= 0.0) {
        return false;
    }
    const double obs_age = std::max(0.0, current_time - latest_seen);
    return obs_age <= credit_time;
}

bool TargetSelector::is_trackable_target(
    const predictor::TargetState& target,
    double dt,
    double current_time
) const
{
    return has_visible_armor(target, dt) || can_track_without_visible(target, current_time);
}

void TargetSelector::try_catch_target(
    const VisibleCandidate& candidate,
    const predictor::BattlefieldSnapshot& snapshot,
    double current_time
)
{
    if (!candidate.valid()) {
        return;
    }

    const auto* current_target = snapshot.find_target(current_target_id_);
    if (current_target == nullptr) {
        current_target_id_ = candidate.target_id;
        target_caught_time_ = current_time;
        return;
    }

    if (candidate.target_id == current_target_id_) {
        target_caught_time_ = current_time;
        return;
    }

    const double keep_time = keep_as_target_time(*current_target);
    if (current_time - target_caught_time_ <= keep_time) {
        return;
    }

    current_target_id_ = candidate.target_id;
    target_caught_time_ = current_time;
}

int TargetSelector::latched_target(double current_time) const
{
    if (current_target_id_ < 0) {
        return -1;
    }
    const double memorizing_time = get_param_or(
        "AutoAim.FireControl.TargetSelector.memorizing_time", 5.0
    );
    if (current_time - target_caught_time_ > memorizing_time) {
        return -1;
    }
    return current_target_id_;
}

double TargetSelector::keep_as_target_time(const predictor::TargetState& target) const
{
    const double default_keep = get_param_or(
        "AutoAim.FireControl.TargetSelector.keep_time", 0.1
    );

    switch (target.enemy_type) {
        case predictor::EnemyType::OUTPOST:
            return get_param_or("AutoAim.FireControl.TargetSelector.keep_time_outpost", 0.5);
        case predictor::EnemyType::SENTRY:
            return get_param_or("AutoAim.FireControl.TargetSelector.keep_time_sentry", default_keep);
        case predictor::EnemyType::BASE:
            return get_param_or("AutoAim.FireControl.TargetSelector.keep_time_base", default_keep);
        case predictor::EnemyType::INFANTRY_3:
        case predictor::EnemyType::INFANTRY_4:
        case predictor::EnemyType::INFANTRY_5:
            return get_param_or("AutoAim.FireControl.TargetSelector.keep_time_infantry", default_keep);
        case predictor::EnemyType::HERO:
            return get_param_or("AutoAim.FireControl.TargetSelector.keep_time_hero", default_keep);
        case predictor::EnemyType::ENGINEER:
            return get_param_or("AutoAim.FireControl.TargetSelector.keep_time_engineer", default_keep);
        default:
            return default_keep;
    }
}

TargetSelector::VisibleCandidate TargetSelector::evaluate_visible_candidate(
    int target_id,
    const predictor::TargetState& target,
    const GimbalState& gimbal,
    double dt
) const
{
    VisibleCandidate candidate;
    const int armor_idx = pick_best_visible_armor(target, gimbal, dt);
    if (armor_idx < 0) {
        return candidate;
    }

    candidate.target_id = target_id;
    candidate.armor_idx = armor_idx;
    const Eigen::Vector3d pos = target.predict_armor_position(armor_idx, dt);
    candidate.score = compute_center_distance(pos, gimbal);
    return candidate;
}

int TargetSelector::pick_best_visible_armor(
    const predictor::TargetState& target,
    const GimbalState& gimbal,
    double dt
) const
{
    int best_idx = -1;
    double best_score = std::numeric_limits<double>::infinity();

    for (int i = 0; i < target.armor_count; ++i) {
        if (!target.armor_visible(i)) continue;

        const Eigen::Vector3d pos = target.predict_armor_position(i, dt);
        const double score = compute_center_distance(pos, gimbal);
        if (score < best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    return best_idx;
}

}  // namespace autoaim::fire_control
