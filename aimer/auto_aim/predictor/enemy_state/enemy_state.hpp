/**
 * @file enemy_state.hpp
 * @brief 单个敌方目标的原始状态 (存储所有检测到的装甲板)
 *
 * 消抖策略 (参考 rm.cv.fans):
 * 1. 面积比例消抖: 太小的装甲板丢弃
 *    - 已存在装甲板: area < max_area * 0.30 → 丢弃
 *    - 新装甲板: area < max_area * 0.40 → 丢弃
 * 2. 跳变消抖: 与上帧距离过大的丢弃
 * 3. 朝向消抖: z_to_v 过大的丢弃
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ENEMY_STATE_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ENEMY_STATE_HPP__

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/math/math.hpp"

namespace autoaim::predictor {

/**
 * @brief 单个目标的原始状态
 *
 * 存储同一目标的所有装甲板，供运动模型使用：
 * - 非陀螺: 只用 best_armor()
 * - 陀螺: 用所有装甲板估计旋转中心、角速度
 */
class EnemyState {
public:
    EnemyState() = default;

    explicit EnemyState(int target_id, EnemyType type = EnemyType::UNKNOWN)
        : target_id_(target_id), enemy_type_(type) {
        // 根据类型设置装甲板数
        max_armor_num_ = (type == EnemyType::OUTPOST) ? 3 : 4;
    }

    // ==================== 更新 ====================

    /**
     * @brief 更新本目标的装甲板数据
     * @param raw 本帧属于该目标的所有 ArmorObservation
     * @param timestamp 时间戳
     */
    void update(const std::vector<ArmorObservation>& raw, double timestamp) {
        timestamp_ = timestamp;

        // 筛选 (含消抖)
        armors_ = filter(raw, prev_armors_);

        if (!armors_.empty()) {
            last_seen_ = timestamp;
            frame_count_++;
            select_best();

            // 只有检测到才更新历史
            prev_armors_ = armors_;
            prev_timestamp_ = timestamp;
        }
    }

    // ==================== 筛选 (含消抖) ====================

    /**
     * @brief 筛选不合理的装甲板 (含面积消抖)
     */
    std::vector<ArmorObservation> filter(
        const std::vector<ArmorObservation>& raw,
        const std::vector<ArmorObservation>& last
    ) const {
        std::vector<ArmorObservation> result;

        // 计算最大面积 (用于面积比例消抖)
        double max_area = 0;
        for (const auto& a : raw) {
            double area = math::get_area(a.pts);
            if (area > max_area) max_area = area;
        }

        // 构建上帧装甲板 ID 集合
        std::set<int> last_ids;
        for (const auto& a : last) {
            last_ids.insert(a.armor_id);
        }

        // 检查是否需要跳变检测
        double dt = timestamp_ - prev_timestamp_;
        bool do_jump_check = !last.empty() && dt > 0 && dt < LOST_TIMEOUT;

        for (const auto& a : raw) {
            if (!a.valid) continue;

            double area = math::get_area(a.pts);

            // 1. 面积比例消抖
            bool is_existing = last_ids.count(a.armor_id) > 0;
            double area_thresh = is_existing
                ? max_area * EXISTING_ARMOR_AREA_RATIO
                : max_area * NEW_ARMOR_AREA_RATIO;
            if (area < area_thresh) continue;

            // 2. 距离检查
            double dist = a.distance();
            if (dist < MIN_DIST || dist > MAX_DIST) continue;

            // 3. 新装甲板距离限制
            if (!is_existing && dist > NEW_ARMOR_MAX_DIST) continue;

            // 4. 朝向检查 (太斜的丢弃)
            if (std::abs(a.z_to_v) > MAX_Z_TO_V) continue;

            // 5. 跳变检查
            if (do_jump_check) {
                double min_jump = min_distance_to(a, last);
                if (min_jump > JUMP_DISTANCE_LIMIT) continue;
            }

            result.push_back(a);
        }

        // 6. 按 z_to_v 排序 (越小越正对)
        std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
            return a.z_to_v < b.z_to_v;
        });

        // 7. 最多保留 max_armor_num 块
        if (result.size() > static_cast<size_t>(max_armor_num_)) {
            result.resize(max_armor_num_);
        }

        // 8. 前哨站专用：按 y 坐标排序 (从左到右)
        if (enemy_type_ == EnemyType::OUTPOST) {
            std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
                return a.pos.y() > b.pos.y();
            });
        }

        return result;
    }

    // ==================== 访问器 ====================

    int target_id() const { return target_id_; }
    EnemyType enemy_type() const { return enemy_type_; }
    double timestamp() const { return timestamp_; }
    double last_seen() const { return last_seen_; }
    int frame_count() const { return frame_count_; }

    void set_enemy_type(EnemyType type) {
        enemy_type_ = type;
        max_armor_num_ = (type == EnemyType::OUTPOST) ? 3 : 4;
    }

    // 所有装甲板
    const std::vector<ArmorObservation>& armors() const { return armors_; }
    int armor_count() const { return static_cast<int>(armors_.size()); }
    bool empty() const { return armors_.empty(); }

    // 最佳装甲板
    int best_index() const { return best_idx_; }
    const ArmorObservation* best_armor() const {
        if (best_idx_ >= 0 && best_idx_ < static_cast<int>(armors_.size())) {
            return &armors_[best_idx_];
        }
        return armors_.empty() ? nullptr : &armors_[0];
    }

    // 上一帧数据 (用于速度估计)
    const std::vector<ArmorObservation>& prev_armors() const { return prev_armors_; }
    double prev_timestamp() const { return prev_timestamp_; }

    // 状态检查
    bool alive(double now, double timeout = 0.5) const {
        return (now - last_seen_) < timeout;
    }

    bool is_spinning() const {
        return armors_.size() >= 2;
    }

    void reset() {
        armors_.clear();
        prev_armors_.clear();
        best_idx_ = -1;
        frame_count_ = 0;
    }

    // ==================== 消抖参数 ====================

    // 面积比例消抖
    static constexpr double EXISTING_ARMOR_AREA_RATIO = 0.30;  // 已存在装甲板
    static constexpr double NEW_ARMOR_AREA_RATIO = 0.40;       // 新装甲板

    // 跳变消抖
    static constexpr double JUMP_DISTANCE_LIMIT = 1.2;  // 最大跳变距离 (m)
    static constexpr double LOST_TIMEOUT = 0.15;        // 丢帧超时 (s)

    // 距离限制
    static constexpr double MIN_DIST = 0.5;             // 最小距离 (m)
    static constexpr double MAX_DIST = 15.0;            // 最大距离 (m)
    static constexpr double NEW_ARMOR_MAX_DIST = 10.0;  // 新装甲板最大距离 (m)

    // 朝向限制
    static constexpr double MAX_Z_TO_V = 1.2;           // 最大朝向角 (~70°)

private:
    void select_best() {
        if (armors_.empty()) {
            best_idx_ = -1;
            return;
        }
        // armors_ 已按 z_to_v 排序，第一个就是最佳
        best_idx_ = 0;
    }

    static double min_distance_to(
        const ArmorObservation& a,
        const std::vector<ArmorObservation>& others
    ) {
        double min_dist = 1e9;
        for (const auto& o : others) {
            double d = (a.pos - o.pos).norm();
            min_dist = std::min(min_dist, d);
        }
        return min_dist;
    }

    // ==================== 数据 ====================

    int target_id_ = 0;
    EnemyType enemy_type_ = EnemyType::UNKNOWN;
    int max_armor_num_ = 4;

    // 当前帧
    std::vector<ArmorObservation> armors_;
    int best_idx_ = -1;
    double timestamp_ = 0;

    // 上一帧 (消抖用)
    std::vector<ArmorObservation> prev_armors_;
    double prev_timestamp_ = 0;

    // 统计
    double last_seen_ = 0;
    int frame_count_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ENEMY_STATE_HPP__
