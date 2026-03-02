// auto_buff fire control types (2026)

#ifndef AIMER_AUTOBUFF_FIRE_CONTROL_TYPES_HPP
#define AIMER_AUTOBUFF_FIRE_CONTROL_TYPES_HPP

#include <Eigen/Core>

#include "aimer/common/fire_control_types.hpp"

namespace autobuff::fire_control {

// 候选目标 (TargetRanker 输出)
struct SlotAimCandidate {
    int slot_id = -1;
    int ccw_rank = -1;   // 逆时针排序位置 (0=第1个)

    bool is_lit = false;
    bool ballistic_valid = false;

    double tracking_error = 1e9;
    float confidence = 0.f;
    double score = -1e9;

    Eigen::Vector3d pred_world = Eigen::Vector3d::Zero();
    ::fire_control::AimResult aim{};
};

}  // namespace autobuff::fire_control

#endif  // AIMER_AUTOBUFF_FIRE_CONTROL_TYPES_HPP
