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
// 主选择函数
// ============================================================================

TargetSelection TargetSelector::select(
    const predictor::BattlefieldSnapshot& snapshot,
    const GimbalState& gimbal,
    double dt)
{
    TargetSelection result;
    double current_time = snapshot.timestamp;

    // 读取参数
    double keep_time = runtime_param::get_param<double>(
        "AutoAim.FireControl.TargetSelector.keep_time"
    );
    double max_angle = runtime_param::get_param<double>(
        "AutoAim.FireControl.TargetSelector.max_angle"
    );

    // ========== 1. 预瞄锁定优先 ==========
    if (forced_target_id_ >= 0 && snapshot.is_valid(forced_target_id_)) {
        const auto& vehicle = snapshot.vehicles[forced_target_id_];

        if (has_visible_armor(vehicle, max_angle, dt, true)) {
            int armor_idx = pick_best_visible_armor(vehicle, gimbal, max_angle, dt);
            if (armor_idx < 0) {
                armor_idx = pick_fallback_armor_idx(vehicle);
            }
            result.has_target = true;
            result.target_id = forced_target_id_;
            result.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);

            current_target_id_ = forced_target_id_;
            last_seen_time_ = current_time;
            return result;
        }
    }

    // ========== 2. 检查当前目标是否还有效 ==========
    bool current_target_valid = false;
    if (current_target_id_ >= 0 && snapshot.is_valid(current_target_id_)) {
        const auto& vehicle = snapshot.vehicles[current_target_id_];

        if (has_visible_armor(vehicle, max_angle, dt, true)) {
            current_target_valid = true;
            last_seen_time_ = current_time;
            int armor_idx = pick_best_visible_armor(vehicle, gimbal, max_angle, dt);
            if (armor_idx < 0) {
                armor_idx = pick_fallback_armor_idx(vehicle);
            }

            // 当前目标仍然有效，保持追踪
            result.has_target = true;
            result.target_id = current_target_id_;
            result.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);
        }
    }

    // ========== 3. 如果当前目标有效，检查是否在保持期内 ==========
    if (current_target_valid) {
        return result;  // 继续追踪当前目标
    }

    // 当前目标无效，检查是否在保持期内
    if (current_target_id_ >= 0 && (current_time - last_seen_time_) < keep_time) {
        // 在保持期内，不切换目标；若模型仍有效则继续输出当前目标
        if (snapshot.is_valid(current_target_id_)) {
            const auto& vehicle = snapshot.vehicles[current_target_id_];
            if (has_visible_armor(vehicle, max_angle, dt, true)) {
                int armor_idx = pick_best_visible_armor(vehicle, gimbal, max_angle, dt);
                if (armor_idx < 0) {
                    armor_idx = pick_fallback_armor_idx(vehicle);
                }
                result.has_target = true;
                result.target_id = current_target_id_;
                result.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);
            }
        }
        return result;
    }

    // ========== 4. 需要选择新目标：选最靠近中心的敌人 ==========
    int best_target_id = -1;
    int best_armor_idx = -1;
    double min_center_dist = std::numeric_limits<double>::infinity();

    snapshot.for_each_valid([&](int id, const predictor::VehicleState& vehicle) {
        if (!vehicle.valid || vehicle.confidence < 0.1) return;

        // 选新目标时只允许“当前可见可打”的装甲板参与竞争，避免幽灵目标抢占
        int armor_idx = pick_best_visible_armor(vehicle, gimbal, max_angle, dt);
        if (armor_idx < 0) {
            return;
        }

        Eigen::Vector3d pos = vehicle.predict_armor_position(armor_idx, dt);
        double dist = compute_center_distance(pos, gimbal);

        if (dist < min_center_dist) {
            min_center_dist = dist;
            best_target_id = id;
            best_armor_idx = armor_idx;
        }
    });

    // ========== 5. 找到了新目标 ==========
    if (best_target_id >= 0) {
        const auto& vehicle = snapshot.vehicles[best_target_id];
        int armor_idx = best_armor_idx;
        if (armor_idx < 0) {
            armor_idx = pick_fallback_armor_idx(vehicle);
        }

        result.has_target = true;
        result.target_id = best_target_id;
        result.predicted_pos = vehicle.predict_armor_position(armor_idx, dt);

        current_target_id_ = best_target_id;
        last_seen_time_ = current_time;
    } else {
        // 没有目标
        current_target_id_ = -1;
    }

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
    last_seen_time_ = 0;
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
