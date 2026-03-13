// 能量机关 YOLOX 后处理实现
// 将 RuneDecoder 输出转换为 BuffDetectionResult

#include "postprocess.hpp"

#include <cmath>
#include <algorithm>

namespace autobuff::detector {

// 扇叶间角间距 (2π / 5 = 72°)
static constexpr double BLADE_ANGLE_STEP = 2.0 * M_PI / autobuff::NUM_SLOTS;

autobuff::BuffDetectionResult Postprocessor::build_result(
    const std::vector<RawRuneObject>& raw_objects,
    const LetterboxMeta& /*meta*/,
    const aimer::RobotState& robot_state,
    int frame_id,
    double timestamp_s,
    autobuff::DetectorBackend backend,
    autobuff::EnemyColor filter_color) const
{
    autobuff::BuffDetectionResult result;
    result.robot_state = robot_state;
    result.frame_id    = frame_id;
    result.timestamp   = timestamp_s;
    result.backend     = backend;

    if (raw_objects.empty()) {
        result.update_summary();
        return result;
    }

    // 颜色过滤: 只保留匹配敌方颜色的检测 (UNKNOWN 不过滤)
    std::vector<const RawRuneObject*> filtered;
    filtered.reserve(raw_objects.size());
    for (const auto& obj : raw_objects) {
        if (filter_color == autobuff::EnemyColor::UNKNOWN ||
            obj.color == filter_color) {
            filtered.push_back(&obj);
        }
    }

    if (filtered.empty()) {
        result.update_summary();
        return result;
    }

    // 1. 估计 R 标中心 (置信度加权平均 + 离群值剔除)
    struct REstimate { cv::Point2f pt; float conf; };
    std::vector<REstimate> r_estimates;
    r_estimates.reserve(filtered.size());

    for (const auto* obj : filtered) {
        r_estimates.push_back({obj->r_center, obj->score});
    }

    if (!r_estimates.empty()) {
        // 离群值剔除: 先算中位数，再剔除距中位数过远的估计
        if (r_estimates.size() >= 3) {
            std::vector<float> xs, ys;
            xs.reserve(r_estimates.size());
            ys.reserve(r_estimates.size());
            for (const auto& e : r_estimates) {
                xs.push_back(e.pt.x);
                ys.push_back(e.pt.y);
            }
            std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
            std::nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
            cv::Point2f median(xs[xs.size() / 2], ys[ys.size() / 2]);

            constexpr float OUTLIER_THRESH_SQ = 80.f * 80.f;
            r_estimates.erase(
                std::remove_if(r_estimates.begin(), r_estimates.end(),
                    [&](const REstimate& e) {
                        auto d = e.pt - median;
                        return d.x * d.x + d.y * d.y > OUTLIER_THRESH_SQ;
                    }),
                r_estimates.end());
        }

        // 置信度加权平均
        cv::Point2f r_sum(0.f, 0.f);
        float w_sum = 0.f;
        for (const auto& e : r_estimates) {
            r_sum += e.pt * e.conf;
            w_sum += e.conf;
        }

        if (w_sum > 0.f) {
            result.r_center.center = r_sum * (1.f / w_sum);
            result.r_center.valid  = true;
            result.r_center.confidence = w_sum / static_cast<float>(r_estimates.size());
        }
    }

    // 2. 为每个检测分配槽位
    struct SlotCandidate {
        float best_score   = -1.f;
        int   best_obj_idx = -1;
        double angle       = 0.0;
    };
    std::array<SlotCandidate, autobuff::NUM_SLOTS> slot_cands{};

    const cv::Point2f& r_center = result.r_center.center;

    for (int i = 0; i < static_cast<int>(filtered.size()); ++i) {
        const auto& obj = *filtered[i];

        // 装甲板中心 = 四角平均
        cv::Point2f armor_center(0.f, 0.f);
        for (const auto& corner : obj.armor_corners) {
            armor_center += corner;
        }
        armor_center *= 0.25f;

        // 计算角度 (数学坐标系, Y 轴向上 → 对像素 Y 取反)
        double dx = armor_center.x - r_center.x;
        double dy = -(armor_center.y - r_center.y);
        double angle = std::atan2(dy, dx);

        // 归一化到 [0, 2π)
        if (angle < 0.0) angle += 2.0 * M_PI;

        // 槽位编号
        int slot_id = static_cast<int>(std::round(angle / BLADE_ANGLE_STEP))
                      % autobuff::NUM_SLOTS;
        if (slot_id < 0) slot_id += autobuff::NUM_SLOTS;

        // 置信度竞争
        if (obj.score > slot_cands[slot_id].best_score) {
            slot_cands[slot_id].best_score   = obj.score;
            slot_cands[slot_id].best_obj_idx = i;
            slot_cands[slot_id].angle        = angle;
        }
    }

    // 3. 构造 DetectedTarget
    for (int s = 0; s < autobuff::NUM_SLOTS; ++s) {
        if (slot_cands[s].best_obj_idx < 0) continue;

        const auto& obj = *filtered[slot_cands[s].best_obj_idx];
        auto& tgt = result.targets[s];

        // 装甲板中心
        cv::Point2f armor_center(0.f, 0.f);
        for (const auto& corner : obj.armor_corners) {
            armor_center += corner;
        }
        armor_center *= 0.25f;

        tgt.center     = armor_center;
        tgt.slot_id    = s;
        tgt.angle      = slot_cands[s].angle;
        tgt.valid      = true;
        tgt.confidence = obj.score;

        // is_lit: INACTIVATED = 当前目标 (需要打击) = lit
        tgt.is_lit = (obj.type == RuneType::INACTIVATED);

        // 填充关键点: 4 个装甲板角点
        // keypoints[0] = bottom_left  (靠近R, 左)
        // keypoints[1] = top_left     (远离R, 左)
        // keypoints[2] = top_right    (远离R, 右)
        // keypoints[3] = bottom_right (靠近R, 右)
        for (int k = 0; k < 4; ++k) {
            tgt.keypoints[k] = obj.armor_corners[k];
        }
        tgt.keypoint_count = 4;

        // landmarks (调试用, 与 keypoints 相同)
        tgt.landmarks.clear();
        tgt.landmarks.reserve(4);
        for (int k = 0; k < 4; ++k) {
            tgt.landmarks.push_back(obj.armor_corners[k]);
        }
    }

    // 4. 更新统计摘要
    result.update_summary();

    return result;
}

}  // namespace autobuff::detector
