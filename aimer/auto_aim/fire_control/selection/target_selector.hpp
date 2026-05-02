/**
 * @file target_selector.hpp
 * @brief 目标选择器 - 从多个敌人中选择最优打击对象
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
 *   - 支持预瞄锁定 (右键按下时强制锁定)
 *
 * 参数通过 runtime_param::get_param 实时获取
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_TARGET_SELECTOR_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_TARGET_SELECTOR_HPP__

#include <limits>
#include <utility>

#include "aimer/auto_aim/fire_control/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

/**
 * @brief 目标选择器
 */
class TargetSelector {
public:
    TargetSelector() = default;

    /**
     * @brief 选择目标
     * @param snapshot 战场快照
     * @param gimbal 当前云台状态 (用于计算转动代价)
     * @param dt 从快照时刻到当前的时间差 (用于插值)
     * @return 选择结果
     */
    TargetSelection select(
        const predictor::BattlefieldSnapshot& snapshot,
        const GimbalState& gimbal,
        double dt
    );

    /**
     * @brief 强制锁定目标 (预瞄锁定)
     */
    void force_lock(int target_id);

    /**
     * @brief 解除预瞄锁定 (保留当前目标)
     */
    void unlock();

    /**
     * @brief 完全清除目标
     */
    void clear_target();

    /**
     * @brief 是否处于锁定状态
     */
    bool is_locked() const { return forced_target_id_ >= 0; }

    /**
     * @brief 获取当前目标ID
     */
    int current_target() const { return current_target_id_; }

private:
    struct VisibleCandidate {
        int target_id = -1;
        int armor_idx = -1;
        double score = std::numeric_limits<double>::infinity();

        bool valid() const { return target_id >= 0 && armor_idx >= 0; }
    };

    // ==================== 辅助方法 ====================

    /**
     * @brief 计算装甲板到图像中心的距离 (用于选敌人)
     *
     * 注意: 这里用 yaw/pitch 角度作为"图像中心距离"的代理
     * 因为当前云台指向就是图像中心
     */
    double compute_center_distance(
        const Eigen::Vector3d& pos,
        const GimbalState& gimbal
    ) const;

    /**
     * @brief 从3D位置计算瞄准角度 (yaw, pitch)
     */
    std::pair<double, double> pos_to_yaw_pitch(const Eigen::Vector3d& pos) const;

    /**
     * @brief 检查该敌人是否有可见装甲板
     */
    bool has_visible_armor(
        const predictor::TargetState& target,
        double dt
    ) const;

    /**
     * @brief 是否允许在“无可见板”时继续跟踪（供 INDIRECT 使用）
     *
     * 对齐 rm.cv.fans 的 top credit 语义：
     *   只要处于陀螺级别且仍在信用时间内，即便窗口角为 0 也可继续跟踪。
     */
    bool can_track_without_visible(
        const predictor::TargetState& target,
        double current_time
    ) const;

    /**
     * @brief 目标是否可用于火控（可见 direct 或无可见但可 indirect）
     */
    bool is_trackable_target(
        const predictor::TargetState& target,
        double dt,
        double current_time
    ) const;

    /**
     * @brief 按 rm.cv.fans TargetCatcher 思路，尝试捕获候选目标
     */
    void try_catch_target(
        const VisibleCandidate& candidate,
        const predictor::BattlefieldSnapshot& snapshot,
        double current_time
    );

    /**
     * @brief 根据 memorizing_time 判断当前目标是否过期
     */
    int latched_target(double current_time) const;

    /**
     * @brief 获取当前目标保持时间 (按 enemy_type 区分)
     */
    double keep_as_target_time(const predictor::TargetState& target) const;

    /**
     * @brief 评估单个敌人的最佳可见候选板（仅 visible）
     */
    VisibleCandidate evaluate_visible_candidate(
        int target_id,
        const predictor::TargetState& target,
        const GimbalState& gimbal,
        double dt
    ) const;

    /**
     * @brief 选择该敌人中最靠近喵中心的可见装甲板
     * @return 装甲板索引，若无可见可打装甲板则返回 -1
     */
    int pick_best_visible_armor(
        const predictor::TargetState& target,
        const GimbalState& gimbal,
        double dt
    ) const;

    // ==================== 状态 ====================

    int current_target_id_ = -1;
    int forced_target_id_ = -1;       // 预瞄锁定的目标
    double target_caught_time_ = 0.0; // 最近一次确认当前目标的时刻
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_TARGET_SELECTOR_HPP__
