/**
 * @file target_selector.cpp
 * @brief 目标选择器实现
 *
 * 职责: 只选"打哪个敌人"，不选装甲板
 *       装甲板选择由 SpinAim 负责
 *
 * 选择策略 (参考 rm.cv.fans):
 *   选敌人：最靠近图像中心 (操作手意图)
 *
 * 锁定机制:
 *   - 当有目标且可见时，保持当前目标
 *   - 当目标丢失超过 keep_time 后才切换
 */

#include "target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

int pick_fallback_armor_idx(const predictor::VehicleState& vehicle) {
    if (vehicle.armor_count <= 0) return -1;

    if (vehicle.recommended_armor_idx >= 0
        && vehicle.recommended_armor_idx < vehicle.armor_count)
    {
        return vehicle.recommended_armor_idx;
    }

    for (int i = 0; i < vehicle.armor_count; ++i) {
        if (vehicle.armors[i].visible) return i;
    }

    return 0;
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
    const double max_angle = runtime_param::get_param<double>(
        "AutoAim.FireControl.TargetSelector.max_angle"
    );

    // ========== 1. 预瞄锁定优先 ==========
    if (forced_target_id_ >= 0 && snapshot.is_valid(forced_target_id_)) {
        const auto& vehicle = snapshot.vehicles[forced_target_id_];
        if (has_visible_armor(vehicle, max_angle, dt, true)) {
            const auto forced_candidate = evaluate_visible_candidate(
                forced_target_id_, vehicle, gimbal, max_angle, dt
            );
            int armor_idx = forced_candidate.valid()
                ? forced_candidate.armor_idx
                : pick_fallback_armor_idx(vehicle);
            if (armor_idx >= 0) {
                current_target_id_ = forced_target_id_;
                target_caught_time_ = current_time;
                if (forced_candidate.valid()) {
                    current_target_score_ = forced_candidate.score;
                }

                result.has_target = true;
                result.target_id = forced_target_id_;
                result.priority = forced_candidate.valid()
                    ? (1.0 / (1.0 + forced_candidate.score))
                    : 0.0;
                result.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);
                return result;
            }
        }
    }

    // ========== 2. 构造“最中心可见目标”候选 ==========
    VisibleCandidate best_candidate;
    snapshot.for_each_valid([&](int id, const predictor::VehicleState& vehicle) {
        if (!vehicle.valid || vehicle.confidence < 0.1) return;
        auto candidate = evaluate_visible_candidate(id, vehicle, gimbal, max_angle, dt);
        if (!candidate.valid()) return;

        // 置信度衰减：同样中心距离下优先更稳定的目标
        candidate.score /= std::max(0.1, vehicle.confidence);
        if (candidate.score < best_candidate.score) {
            best_candidate = candidate;
        }
    });

    // ========== 3. 按 TargetCatcher 逻辑更新锁存目标 ==========
    const int tracked_target = latched_target(current_time);
    if (tracked_target < 0) {
        current_target_id_ = -1;
        current_target_score_ = std::numeric_limits<double>::infinity();
    }
    if (tracked_target >= 0 && snapshot.is_valid(tracked_target)) {
        const auto& tracked_vehicle = snapshot.vehicles[tracked_target];
        if (has_visible_armor(tracked_vehicle, max_angle, dt, true)) {
            // 当前目标仍可打，持续“抓取”当前目标，抑制抖动切换
            target_caught_time_ = current_time;
            auto tracked_visible = evaluate_visible_candidate(
                tracked_target, tracked_vehicle, gimbal, max_angle, dt
            );
            if (tracked_visible.valid()) {
                current_target_score_ = tracked_visible.score;
            }
        } else if (best_candidate.valid()) {
            try_catch_target(
                best_candidate, snapshot, current_time, gimbal, max_angle, dt
            );
        }
    } else if (best_candidate.valid()) {
        try_catch_target(
            best_candidate, snapshot, current_time, gimbal, max_angle, dt
        );
    }

    // ========== 4. 输出当前锁存目标 ==========
    const int selected_target = latched_target(current_time);
    if (selected_target < 0 || !snapshot.is_valid(selected_target)) {
        return result;
    }

    const auto& selected_vehicle = snapshot.vehicles[selected_target];
    if (!has_visible_armor(selected_vehicle, max_angle, dt, true)) {
        return result;
    }

    const auto selected_visible = evaluate_visible_candidate(
        selected_target, selected_vehicle, gimbal, max_angle, dt
    );
    int armor_idx = selected_visible.valid()
        ? selected_visible.armor_idx
        : pick_fallback_armor_idx(selected_vehicle);
    if (armor_idx < 0) {
        return result;
    }

    result.has_target = true;
    result.target_id = selected_target;
    result.priority = selected_visible.valid()
        ? (1.0 / (1.0 + selected_visible.score))
        : 0.0;
    result.predicted_pos = selected_vehicle.predict_armor_position(armor_idx, dt);
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
    current_target_score_ = std::numeric_limits<double>::infinity();
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
    const predictor::VehicleState& vehicle,
    double max_angle,
    double /* dt */,
    bool allow_indirect
) const
{
    for (int i = 0; i < vehicle.armor_count; ++i) {
        const auto& armor = vehicle.armors[i];

        // 仅可见装甲板可参与 DIRECT 可打判定
        if (!armor.visible) continue;

        // 角度过滤 (侧面装甲板不打)
        if (std::abs(armor.z_to_v) > max_angle) continue;

        return true;  // 有可打击的装甲板
    }

    // 仅在“保持/锁定已有目标”时允许陀螺 INDIRECT 兜底
    if (allow_indirect) {
        const double indirect_min_omega = get_param_or(
            "AutoAim.FireControl.TargetSelector.indirect_min_omega", 0.5
        );
        if (vehicle.spin.active && std::abs(vehicle.spin.omega) >= indirect_min_omega
            && vehicle.armor_count > 0)
        {
            return true;
        }
    }

    return false;
}

void TargetSelector::try_catch_target(
    const VisibleCandidate& candidate,
    const predictor::BattlefieldSnapshot& snapshot,
    double current_time,
    const GimbalState& gimbal,
    double max_angle,
    double dt
)
{
    if (!candidate.valid()) {
        return;
    }

    if (current_target_id_ < 0 || !snapshot.is_valid(current_target_id_)) {
        current_target_id_ = candidate.target_id;
        target_caught_time_ = current_time;
        current_target_score_ = candidate.score;
        return;
    }

    if (candidate.target_id == current_target_id_) {
        target_caught_time_ = current_time;
        current_target_score_ = candidate.score;
        return;
    }

    const auto& current_vehicle = snapshot.vehicles[current_target_id_];
    const double keep_time = keep_as_target_time(current_vehicle);
    if (current_time - target_caught_time_ <= keep_time) {
        return;
    }

    double baseline_score = current_target_score_;
    auto current_visible = evaluate_visible_candidate(
        current_target_id_, current_vehicle, gimbal, max_angle, dt
    );
    if (current_visible.valid()) {
        baseline_score = current_visible.score / std::max(0.1, current_vehicle.confidence);
    }

    // 候选必须明显更好才切换，避免多车并行场景下左右跳
    const double switch_hysteresis = std::clamp(
        get_param_or("AutoAim.FireControl.TargetSelector.switch_hysteresis", 0.15),
        0.0, 0.95
    );
    if (std::isfinite(baseline_score)) {
        if (baseline_score <= 1e-6) {
            return;
        }
        const double improve_ratio = (baseline_score - candidate.score) / baseline_score;
        if (improve_ratio < switch_hysteresis) {
            return;
        }
    }

    current_target_id_ = candidate.target_id;
    target_caught_time_ = current_time;
    current_target_score_ = candidate.score;
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

double TargetSelector::keep_as_target_time(const predictor::VehicleState& vehicle) const
{
    const double default_keep = get_param_or(
        "AutoAim.FireControl.TargetSelector.keep_time", 0.1
    );

    switch (vehicle.enemy_type) {
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
    const predictor::VehicleState& vehicle,
    const GimbalState& gimbal,
    double max_angle,
    double dt
) const
{
    VisibleCandidate candidate;
    const int armor_idx = pick_best_visible_armor(vehicle, gimbal, max_angle, dt);
    if (armor_idx < 0) {
        return candidate;
    }

    candidate.target_id = target_id;
    candidate.armor_idx = armor_idx;
    const Eigen::Vector3d pos = vehicle.predict_armor_position(armor_idx, dt);
    candidate.score = compute_center_distance(pos, gimbal);
    return candidate;
}

int TargetSelector::pick_best_visible_armor(
    const predictor::VehicleState& vehicle,
    const GimbalState& gimbal,
    double max_angle,
    double dt
) const
{
    int best_idx = -1;
    double best_dist = std::numeric_limits<double>::infinity();

    for (int i = 0; i < vehicle.armor_count; ++i) {
        const auto& armor = vehicle.armors[i];
        if (!armor.visible) continue;
        if (std::abs(armor.z_to_v) > max_angle) continue;

        const Eigen::Vector3d pos = vehicle.predict_armor_position(i, dt);
        const double dist = compute_center_distance(pos, gimbal);
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
        }
    }

    return best_idx;
}

}  // namespace autoaim::fire_control
