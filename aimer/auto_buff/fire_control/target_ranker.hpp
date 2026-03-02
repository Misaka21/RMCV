// auto_buff TargetRanker (2026)

#ifndef AIMER_AUTOBUFF_FIRE_CONTROL_TARGET_RANKER_HPP
#define AIMER_AUTOBUFF_FIRE_CONTROL_TARGET_RANKER_HPP

#include <vector>

#include "aimer/auto_buff/fire_control/types.hpp"
#include "aimer/auto_buff/predictor/types.hpp"
#include "aimer/common/fire_control_types.hpp"

namespace autobuff::fire_control {

/**
 * @brief 候选目标排序器
 *
 * 为每个 lit slot (或 fallback: 所有 valid slot) 做弹道解算，
 * 计算跟踪误差，并按得分降序排列。
 */
class TargetRanker {
public:
    /**
     * @brief 构建候选列表
     * @param snap   当前战场快照
     * @param latency 延迟信息
     * @param gimbal  当前云台状态
     * @return 按得分降序排列的候选列表
     */
    std::vector<SlotAimCandidate> build(
        const autobuff::predictor::BuffSnapshot& snap,
        const ::fire_control::LatencyInfo& latency,
        const ::fire_control::GimbalState& gimbal) const;

private:
    static double normalize_angle(double a);
};

}  // namespace autobuff::fire_control

#endif  // AIMER_AUTOBUFF_FIRE_CONTROL_TARGET_RANKER_HPP
