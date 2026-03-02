// auto_buff TargetRanker implementation (2026)

#include "target_ranker.hpp"

#include <algorithm>
#include <cmath>

#include "aimer/common/trajectory/solver_factory.hpp"

namespace autobuff::fire_control {

double TargetRanker::normalize_angle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

std::vector<SlotAimCandidate> TargetRanker::build(
    const autobuff::predictor::BuffSnapshot& snap,
    const ::fire_control::LatencyInfo& latency,
    const ::fire_control::GimbalState& gimbal) const
{
    // 收集候选 slot：优先 lit，fallback 到所有 valid
    std::vector<int> candidate_ids;
    candidate_ids.reserve(NUM_SLOTS);
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (snap.is_lit(i)) candidate_ids.push_back(i);
    }
    if (candidate_ids.empty()) {
        for (int i = 0; i < NUM_SLOTS; ++i) {
            if (snap.has_slot(i)) candidate_ids.push_back(i);
        }
    }

    const double predict_dt = latency.prediction_latency();
    const double bullet_speed = snap.self_state.bullet_speed;

    std::vector<SlotAimCandidate> result;
    result.reserve(candidate_ids.size());

    for (int slot : candidate_ids) {
        SlotAimCandidate cand;
        cand.slot_id = slot;
        cand.is_lit = snap.is_lit(slot);
        cand.confidence = snap.slots[slot].confidence;

        // 预测世界坐标
        cand.pred_world = snap.predict_slot_world(slot, predict_dt);

        // 弹道解算
        cand.aim = ::fire_control::trajectory::solve(cand.pred_world, bullet_speed);
        cand.ballistic_valid = cand.aim.valid;

        // 跟踪误差
        if (cand.aim.valid) {
            double dy = normalize_angle(cand.aim.yaw - gimbal.yaw);
            double dp = cand.aim.pitch - gimbal.pitch;
            cand.tracking_error = std::hypot(dy, dp);
        }

        // 得分 (跟踪误差越小越好)
        cand.score = cand.ballistic_valid ? -cand.tracking_error : -1e9;

        // ccw_rank: 在 snap.ccw_lit_rank 数组中查找该 slot 的逆时针排序位置
        cand.ccw_rank = -1;
        for (int r = 0; r < snap.ranked_count; ++r) {
            if (snap.ccw_lit_rank[r] == slot) {
                cand.ccw_rank = r;
                break;
            }
        }

        result.push_back(cand);
    }

    // 按得分降序排列
    std::sort(result.begin(), result.end(),
              [](const SlotAimCandidate& a, const SlotAimCandidate& b) {
                  return a.score > b.score;
              });

    return result;
}

}  // namespace autobuff::fire_control
