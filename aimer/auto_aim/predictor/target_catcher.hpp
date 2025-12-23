/**
 * @file target_catcher.hpp
 * @brief 目标捕获器 - 目标切换消抖
 *
 * 参考 rm.cv.fans 的 TargetCatcher:
 * - 不同车型有不同的保持时间
 * - 普通步兵: 0.1s
 * - 平衡步兵/前哨站: 0.5s (装甲板容易消失)
 * - 目标记忆时间: 5s (丢失后仍保留)
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_TARGET_CATCHER_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_TARGET_CATCHER_HPP__

#include <array>
#include <unordered_map>

#include "types.hpp"

namespace autoaim::predictor {

/**
 * @brief 目标捕获器 - 延迟切换目标
 *
 * 防止目标频繁切换导致的抖动
 */
class TargetCatcher {
public:
    TargetCatcher() = default;

    /**
     * @brief 尝试捕获目标
     *
     * 如果当前无目标，立即捕获
     * 如果有目标但想切换到其他目标，需要等待当前目标的保持时间
     */
    void try_catch(int target_id, EnemyType type, double timestamp) {
        if (target_id <= 0 || target_id > MAX_TARGETS) return;

        if (current_target_ == -1) {
            // 无目标，立即捕获
            current_target_ = target_id;
            current_type_ = type;
            caught_time_ = timestamp;
        } else if (current_target_ != target_id) {
            // 想切换到其他目标，检查保持时间
            double keep_time = get_keep_time(current_type_);
            if (timestamp - caught_time_ > keep_time) {
                // 超过保持时间，允许切换
                current_target_ = target_id;
                current_type_ = type;
                caught_time_ = timestamp;
            }
        } else {
            // 相同目标，更新时间
            caught_time_ = timestamp;
        }
    }

    /**
     * @brief 获取当前目标
     * @return 目标 ID，-1 表示无目标
     */
    int get_target(double timestamp) const {
        if (current_target_ == -1) return -1;

        // 检查是否超过记忆时间
        if (timestamp - caught_time_ > MEMORY_TIME) {
            return -1;
        }

        return current_target_;
    }

    /**
     * @brief 强制设置目标 (跳过消抖)
     */
    void force_set(int target_id, EnemyType type, double timestamp) {
        current_target_ = target_id;
        current_type_ = type;
        caught_time_ = timestamp;
    }

    /**
     * @brief 重置
     */
    void reset() {
        current_target_ = -1;
        current_type_ = EnemyType::UNKNOWN;
        caught_time_ = 0;
    }

    // 保持时间配置 (不同车型)
    static constexpr double KEEP_TIME_DEFAULT = 0.1;      // 普通步兵
    static constexpr double KEEP_TIME_BALANCE = 0.5;      // 平衡步兵
    static constexpr double KEEP_TIME_OUTPOST = 0.5;      // 前哨站
    static constexpr double MEMORY_TIME = 5.0;            // 目标记忆时间

private:
    static double get_keep_time(EnemyType type) {
        switch (type) {
            case EnemyType::OUTPOST:
                return KEEP_TIME_OUTPOST;
            // 平衡步兵需要在运行时判断，这里暂时用默认值
            default:
                return KEEP_TIME_DEFAULT;
        }
    }

    int current_target_ = -1;
    EnemyType current_type_ = EnemyType::UNKNOWN;
    double caught_time_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_TARGET_CATCHER_HPP__
