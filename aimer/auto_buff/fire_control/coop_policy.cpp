// auto_buff CoopPolicy implementation (2026)

#include "coop_policy.hpp"

#include "plugin/param/runtime_parameter.hpp"

namespace autobuff::fire_control {

int CoopPolicy::select(
    const autobuff::predictor::BuffSnapshot& snap,
    const std::vector<SlotAimCandidate>& cands) const
{
    if (cands.empty()) return -1;

    // 最优候选 (已按 score 降序排列)
    auto best_slot_id = [&]() -> int {
        for (const auto& c : cands) {
            if (c.ballistic_valid) return c.slot_id;
        }
        return cands.front().slot_id;
    };

    // 在调用点直接读取参数，禁止缓存
    const std::string role_str = runtime_param::get_param<std::string>("AutoBuff.FireControl.coop_role");
    const bool coop_only_large = runtime_param::get_param<bool>("AutoBuff.FireControl.coop_only_large_active");

    CoopRole role = parse_coop_role(role_str);

    // 协同模式关闭
    if (role == CoopRole::DISABLED) {
        return best_slot_id();
    }

    // 仅在大符激活时启用协同
    if (coop_only_large && snap.mode != autobuff::BuffMode::LARGE_ACTIVE) {
        return best_slot_id();
    }

    // 目标排位 (CCW_FIRST=0, CCW_SECOND=1)
    const int target_rank = (role == CoopRole::CCW_FIRST) ? 0 : 1;

    // 查找对应排位且弹道有效的候选
    for (const auto& c : cands) {
        if (c.ccw_rank == target_rank && c.ballistic_valid) {
            return c.slot_id;
        }
    }

    // 找不到对应排位，回退到最优候选
    return best_slot_id();
}

}  // namespace autobuff::fire_control
