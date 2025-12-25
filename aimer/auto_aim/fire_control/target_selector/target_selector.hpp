/**
 * @file target_selector.hpp
 * @brief 目标选择器 - 从多个目标中选择最优打击对象
 *
 * 评分标准:
 *   1. 面积 (大且正对优先)
 *   2. 静止优先 (速度小优先)
 *   3. 距离 (近优先)
 *   4. 目标类型优先级 (英雄 > 步兵 > 哨兵)
 *   5. 置信度
 *
 * 切换策略:
 *   - 迟滞比较，新目标必须明显更好才切换，防止震荡
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_TARGET_SELECTOR_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_TARGET_SELECTOR_HPP__

#include "aimer/auto_aim/fire_control/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

/**
 * @brief 目标选择器
 */
class TargetSelector {
public:
    struct Config {
        // 权重系数
        double w_area = 2.0;               // 面积权重 (最重要)
        double w_still = 1.5;              // 静止权重
        double w_distance = 1.0;           // 距离权重
        double w_priority = 1.0;           // 类型优先级权重
        double w_confidence = 0.5;         // 置信度权重

        // 面积过滤: 小于最大面积 * area_filter_ratio 的排除
        double area_filter_ratio = 0.4;

        // 迟滞切换: 新目标必须比当前目标好 hysteresis 比例才切换
        // 例如 0.15 表示新目标评分必须 > 当前 * 1.15
        double switch_hysteresis = 0.15;
        double switch_armor_hysteresis = 0.10;  // 同一目标内切换装甲板

        // 静止判定
        double still_speed_scale = 2.0;    // 速度影响系数

        // 距离参数
        double max_distance = 10.0;        // 最大有效距离 (m)

        // 角度参数
        double max_angle = 1.2;            // 最大有效角度 (rad, 约 70°)
    };

    explicit TargetSelector(const Config& config = {});

    /**
     * @brief 选择目标
     * @param snapshot 战场快照
     * @param dt 从快照时刻到当前的时间差 (用于插值)
     * @return 选择结果
     */
    TargetSelection select(const predictor::BattlefieldSnapshot& snapshot, double dt);

    /**
     * @brief 强制指定目标 (用于手动锁定)
     */
    void force_target(int target_id);

    /**
     * @brief 清除目标锁定
     */
    void clear_target();

    // 配置
    void set_config(const Config& config) { config_ = config; }
    const Config& config() const { return config_; }

private:
    /**
     * @brief 计算投影面积 (考虑距离和朝向)
     */
    double compute_projected_area(const predictor::ArmorState& armor) const;

    /**
     * @brief 计算目标评分 (不含切换惩罚)
     */
    double compute_score(
        const predictor::VehicleState& vehicle,
        const predictor::ArmorState& armor,
        double max_area
    ) const;

    /**
     * @brief 获取目标类型优先级
     */
    double get_type_priority(predictor::EnemyType type) const;

    /**
     * @brief 选择最佳装甲板 (同一目标内)
     */
    int select_best_armor(
        const predictor::VehicleState& vehicle,
        double max_area
    ) const;

    /**
     * @brief 迟滞判断: 是否应该切换目标
     */
    bool should_switch_target(double new_score, double current_score) const;

    /**
     * @brief 迟滞判断: 是否应该切换装甲板
     */
    bool should_switch_armor(double new_score, double current_score) const;

    Config config_;

    // 状态
    int current_target_id_ = -1;
    int current_armor_idx_ = -1;
    double current_score_ = 0;
    int forced_target_id_ = -1;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_TARGET_SELECTOR_HPP__
