// 能量机关 YOLO 后处理实现
// 将 Sp25Decoder 输出转换为 BuffDetectionResult

#include "postprocess.hpp"

#include <cmath>

namespace autobuff::detector {

// 扇叶间角间距 (2π / 5 = 72°)
static constexpr double BLADE_ANGLE_STEP = 2.0 * M_PI / autobuff::NUM_SLOTS;

// R 标外推系数：kpt[5] 是内侧尖端，kpt[4] 是扇叶中心
// R 中心 ≈ kpt[5] + (kpt[5] - kpt[4]) * 0.4
// 即 R = kpt[5] * 1.4 - kpt[4] * 0.4
// 等价 CLAUDE.md 中描述的 (kpt[5] - kpt[4]) * 1.4 + kpt[4]
// = kpt[5] * 1.4 - kpt[4] * 1.4 + kpt[4] = kpt[5] * 1.4 - kpt[4] * 0.4 ✓
static constexpr float R_EXTRAP_A = 1.4f;   // kpt[5] 系数
static constexpr float R_EXTRAP_B = 0.4f;   // kpt[4] 系数 (减去)

autobuff::BuffDetectionResult Postprocessor::build_result(
    const std::vector<RawBuffObject>& raw_objects,
    const LetterboxMeta& /*meta*/,
    const aimer::RobotState& robot_state,
    int frame_id,
    double timestamp_s,
    autobuff::DetectorBackend backend) const
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

    // 1. 估计 R 标中心 (平均所有检测的外推值)
    //    每个检测提供一个 R 中心估计：R = kpt[5] * 1.4 - kpt[4] * 0.4
    cv::Point2f r_sum(0.f, 0.f);
    int r_count = 0;
    for (const auto& obj : raw_objects) {
        if (obj.kpt_count < 6) {
            continue;
        }
        const cv::Point2f& kpt4 = obj.kpts[4];  // 扇叶中心
        const cv::Point2f& kpt5 = obj.kpts[5];  // 内侧尖端 (指向 R 标)
        cv::Point2f r_est = kpt5 * R_EXTRAP_A - kpt4 * R_EXTRAP_B;
        r_sum += r_est;
        r_count++;
    }

    if (r_count > 0) {
        result.r_center.center = r_sum * (1.f / static_cast<float>(r_count));
        result.r_center.valid  = true;
        result.r_center.confidence = 1.0f;  // 由检测置信度间接保证
    }

    // 2. 为每个检测分配槽位
    //    角度: atan2(-(center.y - r.y), center.x - r.x)  (Y 轴翻转，像素坐标向下)
    //    槽位: round(angle / (2π/5)) mod 5
    //    冲突: 置信度更高者胜

    // 临时槽位: 存储 (最高置信度, 对应的 raw_object 索引)
    struct SlotCandidate {
        float best_score  = -1.f;
        int   best_obj_idx = -1;
        double angle       = 0.0;
    };
    std::array<SlotCandidate, autobuff::NUM_SLOTS> slot_cands{};

    const cv::Point2f& r_center = result.r_center.center;

    for (int i = 0; i < static_cast<int>(raw_objects.size()); ++i) {
        const auto& obj = raw_objects[i];
        if (obj.kpt_count < 6) {
            continue;
        }

        // 扇叶中心为 kpt[4]
        const cv::Point2f& fan_center = obj.kpts[4];

        // 计算角度 (数学坐标系，Y 轴向上 → 对像素 Y 取反)
        double dx = fan_center.x - r_center.x;
        double dy = -(fan_center.y - r_center.y);
        double angle = std::atan2(dy, dx);

        // 归一化到 [0, 2π)
        if (angle < 0.0) {
            angle += 2.0 * M_PI;
        }

        // 槽位编号
        int slot_id = static_cast<int>(std::round(angle / BLADE_ANGLE_STEP))
                      % autobuff::NUM_SLOTS;
        if (slot_id < 0) {
            slot_id += autobuff::NUM_SLOTS;
        }

        // 置信度竞争
        if (obj.score > slot_cands[slot_id].best_score) {
            slot_cands[slot_id].best_score   = obj.score;
            slot_cands[slot_id].best_obj_idx = i;
            slot_cands[slot_id].angle        = angle;
        }
    }

    // 3. 构造 DetectedTarget
    for (int s = 0; s < autobuff::NUM_SLOTS; ++s) {
        if (slot_cands[s].best_obj_idx < 0) {
            continue;
        }

        const auto& obj = raw_objects[slot_cands[s].best_obj_idx];
        auto& tgt       = result.targets[s];

        tgt.center     = obj.kpts[4];
        tgt.slot_id    = s;
        tgt.angle      = slot_cands[s].angle;
        tgt.is_lit     = true;    // YOLO 只识别点亮扇叶
        tgt.valid      = true;
        tgt.confidence = obj.score;

        // 填充关键点
        tgt.keypoints = obj.kpts;
        tgt.keypoint_count = obj.kpt_count;

        // 填充 landmarks (四角，调试用)
        tgt.landmarks.clear();
        tgt.landmarks.reserve(4);
        for (int k = 0; k < 4; ++k) {
            tgt.landmarks.push_back(obj.kpts[k]);
        }
    }

    // 4. 更新统计摘要 (target_count, lit_count, lit_mask, status)
    result.update_summary();

    return result;
}

}  // namespace autobuff::detector
