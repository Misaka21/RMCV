// auto_buff CoopPolicy (2026)

#ifndef AIMER_AUTOBUFF_FIRE_CONTROL_COOP_POLICY_HPP
#define AIMER_AUTOBUFF_FIRE_CONTROL_COOP_POLICY_HPP

#include <string>
#include <vector>

#include "aimer/auto_buff/common/types.hpp"
#include "aimer/auto_buff/fire_control/types.hpp"
#include "aimer/auto_buff/predictor/types.hpp"

namespace autobuff::fire_control {

/**
 * @brief 解析协同角色字符串
 */
inline CoopRole parse_coop_role(const std::string& s) {
    if (s == "ccw_first" || s == "CCW_FIRST") return CoopRole::CCW_FIRST;
    if (s == "ccw_second" || s == "CCW_SECOND") return CoopRole::CCW_SECOND;
    return CoopRole::DISABLED;
}

/**
 * @brief 双车协同策略
 *
 * 根据协同角色选择目标 slot。
 * - DISABLED: 直接返回得分最高的候选
 * - CCW_FIRST: 优先打逆时针第 1 个 lit slot (rank=0)
 * - CCW_SECOND: 优先打逆时针第 2 个 lit slot (rank=1)
 *
 * 注意: 所有参数均在调用点通过 runtime_param::get_param 读取，不缓存。
 */
class CoopPolicy {
public:
    /**
     * @brief 选择目标 slot
     * @param snap   当前战场快照
     * @param cands  已排序的候选列表 (TargetRanker 输出)
     * @return 选中的 slot_id，-1 表示无目标
     */
    int select(
        const autobuff::predictor::BuffSnapshot& snap,
        const std::vector<SlotAimCandidate>& cands) const;
};

}  // namespace autobuff::fire_control

#endif  // AIMER_AUTOBUFF_FIRE_CONTROL_COOP_POLICY_HPP
