// YOLOX 能量机关模型解码器实现

#include "rune_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace autobuff::detector {

static constexpr int NUM_POINTS    = 5;
static constexpr int NUM_POINTS_2  = 2 * NUM_POINTS;  // 10
static constexpr int NUM_COLORS    = 2;
static constexpr int NUM_CLASSES   = 2;
// 每个 anchor 的输出宽度: 10 (关键点) + 1 (conf) + 2 (颜色) + 2 (类型) = 15
static constexpr int ANCHOR_WIDTH  = NUM_POINTS_2 + 1 + NUM_COLORS + NUM_CLASSES;

// NMS merge 参数
static constexpr float MERGE_CONF_ERROR = 0.15f;
static constexpr float MERGE_MIN_IOU    = 0.9f;

// 由于训练失误，网络的颜色是反的: class0=BLUE, class1=RED
static autobuff::EnemyColor decode_color(int color_id) {
    switch (color_id) {
        case 0: return autobuff::EnemyColor::BLUE;
        case 1: return autobuff::EnemyColor::RED;
        default: return autobuff::EnemyColor::UNKNOWN;
    }
}

RuneDecoder::RuneDecoder(float conf_threshold, float nms_threshold,
                         int top_k, int input_w, int input_h)
    : conf_threshold_(conf_threshold)
    , nms_threshold_(nms_threshold)
    , top_k_(top_k)
    , input_w_(input_w)
    , input_h_(input_h)
{
    generate_grids_and_strides();
}

void RuneDecoder::generate_grids_and_strides() {
    std::vector<int> strides = {8, 16, 32};
    grid_strides_.clear();

    for (int stride : strides) {
        int num_w = input_w_ / stride;
        int num_h = input_h_ / stride;
        for (int g1 = 0; g1 < num_h; ++g1) {
            for (int g0 = 0; g0 < num_w; ++g0) {
                grid_strides_.push_back({g0, g1, stride});
            }
        }
    }
}

// 辅助结构: 带 merge 支持的中间检测结果
struct MergeableDetection {
    RawRuneObject obj;
    // 用于 merge 平均的子检测
    std::vector<RawRuneObject> children;
};

static float intersection_area(const cv::Rect& a, const cv::Rect& b) {
    cv::Rect inter = a & b;
    return static_cast<float>(inter.area());
}

std::vector<RawRuneObject> RuneDecoder::decode(
    const float* output_data,
    const std::vector<int64_t>& output_shape,
    const LetterboxMeta& meta) const
{
    if (output_shape.size() < 3) return {};

    int num_anchors = static_cast<int>(output_shape[1]);
    int anchor_dim  = static_cast<int>(output_shape[2]);

    // 安全检查
    if (num_anchors <= 0 || anchor_dim < ANCHOR_WIDTH) return {};
    if (num_anchors != static_cast<int>(grid_strides_.size())) return {};

    // 解码关键点 + 分类
    std::vector<MergeableDetection> proposals;
    proposals.reserve(256);

    for (int idx = 0; idx < num_anchors; ++idx) {
        const float* row = output_data + idx * anchor_dim;

        float confidence = row[NUM_POINTS_2];
        if (confidence < conf_threshold_) continue;

        int grid0  = grid_strides_[idx].grid0;
        int grid1  = grid_strides_[idx].grid1;
        int stride = grid_strides_[idx].stride;

        // 颜色分类 (argmax)
        int color_id = 0;
        float max_color_score = row[NUM_POINTS_2 + 1];
        for (int c = 1; c < NUM_COLORS; ++c) {
            float s = row[NUM_POINTS_2 + 1 + c];
            if (s > max_color_score) {
                max_color_score = s;
                color_id = c;
            }
        }

        // 类型分类 (argmax)
        int class_id = 0;
        float max_class_score = row[NUM_POINTS_2 + 1 + NUM_COLORS];
        for (int c = 1; c < NUM_CLASSES; ++c) {
            float s = row[NUM_POINTS_2 + 1 + NUM_COLORS + c];
            if (s > max_class_score) {
                max_class_score = s;
                class_id = c;
            }
        }

        // 解码 5 个关键点 (letterboxed 坐标 → 原图坐标)
        cv::Point2f kpts_net[NUM_POINTS];
        for (int p = 0; p < NUM_POINTS; ++p) {
            float x = (row[p * 2]     + grid0) * stride;
            float y = (row[p * 2 + 1] + grid1) * stride;
            kpts_net[p] = meta.restore(cv::Point2f(x, y));
        }

        RawRuneObject obj;
        obj.r_center = kpts_net[0];
        for (int k = 0; k < 4; ++k) {
            obj.armor_corners[k] = kpts_net[k + 1];
        }
        obj.color = decode_color(color_id);
        obj.type  = static_cast<RuneType>(class_id);
        obj.score = confidence;

        // 计算包围盒 (用所有关键点)
        std::vector<cv::Point2f> all_pts = {
            kpts_net[0], kpts_net[1], kpts_net[2], kpts_net[3], kpts_net[4]};
        obj.box = cv::boundingRect(all_pts);

        proposals.push_back({obj, {}});
    }

    // TopK: 按置信度降序排列
    std::sort(proposals.begin(), proposals.end(),
        [](const MergeableDetection& a, const MergeableDetection& b) {
            return a.obj.score > b.obj.score;
        });
    if (static_cast<int>(proposals.size()) > top_k_) {
        proposals.resize(top_k_);
    }

    // NMS with merge
    std::vector<int> keep_indices;
    int n = static_cast<int>(proposals.size());
    std::vector<float> areas(n);
    for (int i = 0; i < n; ++i) {
        areas[i] = static_cast<float>(proposals[i].obj.box.area());
    }

    for (int i = 0; i < n; ++i) {
        bool keep = true;
        for (int j : keep_indices) {
            float inter = intersection_area(proposals[i].obj.box, proposals[j].obj.box);
            float union_area = areas[i] + areas[j] - inter;
            float iou = inter / union_area;

            if (iou > nms_threshold_ || std::isnan(iou)) {
                keep = false;
                // Merge: 同类型同颜色、高IoU、相近置信度的检测做平均
                if (proposals[i].obj.type == proposals[j].obj.type &&
                    proposals[i].obj.color == proposals[j].obj.color &&
                    iou > MERGE_MIN_IOU &&
                    std::abs(proposals[i].obj.score - proposals[j].obj.score) < MERGE_CONF_ERROR) {
                    proposals[j].children.push_back(proposals[i].obj);
                }
                break;
            }
        }
        if (keep) {
            keep_indices.push_back(i);
        }
    }

    // 构建最终结果 (merge 平均)
    std::vector<RawRuneObject> results;
    results.reserve(keep_indices.size());

    for (int idx : keep_indices) {
        auto& det = proposals[idx];
        if (!det.children.empty()) {
            float count = static_cast<float>(det.children.size() + 1);
            // 累加所有子检测
            cv::Point2f r_sum = det.obj.r_center;
            std::array<cv::Point2f, 4> corner_sum = det.obj.armor_corners;
            for (const auto& child : det.children) {
                r_sum += child.r_center;
                for (int k = 0; k < 4; ++k) {
                    corner_sum[k] += child.armor_corners[k];
                }
            }
            det.obj.r_center = r_sum * (1.f / count);
            for (int k = 0; k < 4; ++k) {
                det.obj.armor_corners[k] = corner_sum[k] * (1.f / count);
            }
        }
        results.push_back(det.obj);
    }

    return results;
}

}  // namespace autobuff::detector
