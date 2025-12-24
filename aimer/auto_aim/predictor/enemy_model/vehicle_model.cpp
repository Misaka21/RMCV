/**
 * @file vehicle_model.cpp
 * @brief 车辆运动模型实现
 *
 * 过滤逻辑参考 rm.cv.fans/aimer/auto_aim/predictor/enemy/enemy_state.cpp
 */

#include "vehicle_model.hpp"

#include <algorithm>
#include <cmath>

#include <fmt/color.h>

#include "aimer/common/math/math.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

// ============================================================================
// 辅助函数: 读取运行时参数
// ============================================================================

namespace {

// 默认参数值
constexpr double DEFAULT_EXISTING_ARMOR_AREA_RATIO = 0.30;
constexpr double DEFAULT_NEW_ARMOR_AREA_RATIO = 0.40;
constexpr double DEFAULT_JUMP_DISTANCE_LIMIT = 1.2;
constexpr double DEFAULT_NEW_ARMOR_MAX_DIST = 10.0;
constexpr double DEFAULT_LOST_TIMEOUT = 0.5;
constexpr double DEFAULT_ARMOR_CREDIT_TIME = 0.1;

// 辅助函数: 安全读取运行时参数，找不到时返回默认值
double get_double_param(const std::string& name, double default_val) {
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (auto* val = std::get_if<double>(&*ptr)) {
            return *val;
        }
    }
    return default_val;
}

double get_existing_armor_area_ratio() {
    return get_double_param("AutoAim.Predictor.existing_armor_area_ratio", DEFAULT_EXISTING_ARMOR_AREA_RATIO);
}

double get_new_armor_area_ratio() {
    return get_double_param("AutoAim.Predictor.new_armor_area_ratio", DEFAULT_NEW_ARMOR_AREA_RATIO);
}

double get_jump_distance_limit() {
    return get_double_param("AutoAim.Predictor.jump_distance_limit", DEFAULT_JUMP_DISTANCE_LIMIT);
}

double get_new_armor_max_distance() {
    return get_double_param("AutoAim.Predictor.new_armor_max_distance", DEFAULT_NEW_ARMOR_MAX_DIST);
}

double get_lost_timeout() {
    return get_double_param("AutoAim.Predictor.lost_timeout", DEFAULT_LOST_TIMEOUT);
}

double get_armor_credit_time() {
    return get_double_param("AutoAim.Predictor.armor_credit_time", DEFAULT_ARMOR_CREDIT_TIME);
}

}  // namespace

// ============================================================================
// VehicleModel 实现
// ============================================================================

VehicleModel::VehicleModel(int target_id, EnemyType enemy_type)
    : target_id_(target_id),
      enemy_type_(enemy_type),
      armor_model_(get_armor_credit_time()),
      spin_model_(enemy_type == EnemyType::OUTPOST ? 3 : 4) {}

void VehicleModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    ++frame_count_;

    // 1. 消抖过滤 (参考 rm.cv.fans screened_armors)
    auto filtered = filter(observations, prev_armors_);

    if (filtered.empty()) {
        if (initialized_ && (timestamp - last_update_time_) > get_lost_timeout()) {
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

    // 4. SpinModel: 整车 EKF 滤波 (用最正对的装甲板)
    if (!filtered.empty()) {
        spin_model_.update(filtered[0], timestamp);
    }

    // DEBUG: 输出装甲板数量和 ID
    if (armors_with_id.size() > 1 || armor_model_.size() > 1) {
        fmt::print(fmt::fg(fmt::color::orange),
            "[T{}] obs:{} filtered:{} active:{} filters:{}\n",
            target_id_, observations.size(), filtered.size(),
            armors_with_id.size(), armor_model_.size());
        for (const auto& a : armors_with_id) {
            fmt::print("  armor id={} pos=({:.2f},{:.2f},{:.2f})\n",
                a.id, a.pos().x(), a.pos().y(), a.pos().z());
        }
    }

    if (!initialized_) {
        initialized_ = true;
    }

    // 5. 更新陀螺状态
    spin_.omega = spin_model_.get_omega();
    spin_.phase = spin_model_.get_theta();
    spin_.radius = spin_model_.get_radius();
    spin_.update_level(spin_.omega);

    prev_armors_ = filtered;
    prev_timestamp_ = timestamp;
}

/**
 * @brief 过滤无效观测 (参考 rm.cv.fans EnemyState::screened_armors)
 *
 * 过滤规则:
 * 1. 面积过小: 已存在装甲板 < max_area * 0.30, 新装甲板 < max_area * 0.40
 * 2. 距离过远: 新装甲板 > 10.0m
 * 3. 跳变过大: 相邻帧位置跳变 > 1.2m
 *
 * 注意: 不检查 MIN_DIST 和 z_to_v (与 rm.cv.fans 保持一致)
 */
std::vector<ArmorObservation> VehicleModel::filter(
    const std::vector<ArmorObservation>& raw,
    const std::vector<ArmorObservation>& last
) const {
    std::vector<ArmorObservation> result;

    if (raw.empty()) return result;

    // 读取运行时参数
    const double existing_area_ratio = get_existing_armor_area_ratio();
    const double new_area_ratio = get_new_armor_area_ratio();
    const double jump_limit = get_jump_distance_limit();
    const double new_max_dist = get_new_armor_max_distance();

    // 找最大面积
    double max_area = 0;
    for (const auto& a : raw) {
        if (!a.valid) continue;
        double area = math::get_area(a.pts);
        if (area > max_area) max_area = area;
    }

    for (const auto& a : raw) {
        if (!a.valid) continue;

        double area = math::get_area(a.pts);

        // 找到与上一帧最近的距离
        double closest = 1e9;
        for (const auto& o : last) {
            double d = (a.pos - o.pos).norm();
            if (d < closest) closest = d;
        }

        // 规则1: 相邻帧跳变过大
        if (!last.empty() && closest > jump_limit) {
            continue;
        }

        // 判断是否是已存在的装甲板 (用位置匹配，因为 ID 还未分配)
        bool is_existing = !last.empty() && closest < 0.5;

        // 规则2: 面积过小
        if (is_existing) {
            if (area < max_area * existing_area_ratio) {
                continue;
            }
        } else {
            // 新装甲板
            if (area < max_area * new_area_ratio) {
                continue;
            }
            // 规则3: 新装甲板距离过远
            if (a.distance() > new_max_dist) {
                continue;
            }
        }

        result.push_back(a);
    }

    // 按 z_to_v 排序 (正对的优先)
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        return a.z_to_v < b.z_to_v;
    });

    // 最多保留 4 块装甲板
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
    vs.spin = spin_;

    if (!initialized_) return vs;

    double dt = timestamp - last_update_time_;

    // 根据陀螺等级选择模型
    if (spin_model_.get_spin_level() >= SpinLevel::LOW && spin_model_.valid()) {
        // ========== 陀螺模式: 用 SpinModel ==========
        vs.center = spin_model_.predict_center(dt);
        vs.velocity = spin_model_.get_velocity();

        // 预测所有装甲板位置
        int armor_num = (enemy_type_ == EnemyType::OUTPOST) ? 3 : 4;
        vs.armor_count = armor_num;

        double best_score = -1;
        int best_idx = -1;

        for (int i = 0; i < armor_num; ++i) {
            auto& as = vs.armors[i];
            as.id = i;
            as.position = spin_model_.predict_armor_pos(i, dt);
            as.velocity = vs.velocity;  // 近似用中心速度
            as.yaw = spin_model_.get_theta() + i * (2.0 * M_PI / armor_num);
            as.visible = (i == 0);  // 只有当前追踪的可见
            as.last_seen = last_update_time_;

            // 评分: 越正对越好 (用 cos(装甲板朝向 - 视线方向))
            double armor_yaw = as.yaw;
            double view_yaw = std::atan2(as.position.y(), as.position.x());
            double angle_diff = std::abs(math::reduced_angle(armor_yaw - view_yaw - M_PI));
            as.score = std::cos(angle_diff);

            if (as.score > best_score) {
                best_score = as.score;
                best_idx = i;
            }
        }

        vs.recommended_armor_idx = best_idx;

    } else {
        // ========== 普通模式: 用 ArmorModel ==========
        auto armor_states = armor_model_.get_armor_states(timestamp);

        vs.armor_count = static_cast<int>(std::min(armor_states.size(), size_t(MAX_ARMORS_PER_TARGET)));

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
    }

    return vs;
}

bool VehicleModel::alive() const {
    return initialized_;
}

void VehicleModel::reset() {
    initialized_ = false;
    identifier_.reset();
    armor_model_.reset();
    spin_model_.reset();
    prev_armors_.clear();
    spin_.reset();
    frame_count_ = 0;
}

}  // namespace autoaim::predictor
