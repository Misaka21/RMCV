// sp25 模型解码器实现
// 输出格式: [1, 17, 8400] 或 [1, 8400, 17]
// 17 = 4 box(cx,cy,w,h) + 1 score + 6 keypoints * 2

#include "buff_decoder.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/dnn.hpp>

namespace autobuff::detector {

// sp25 单类别输出特征维度
namespace sp25 {
    // 特征向量布局 (每个候选框)
    constexpr int BOX_CX   = 0;  // center_x
    constexpr int BOX_CY   = 1;  // center_y
    constexpr int BOX_W    = 2;  // width
    constexpr int BOX_H    = 3;  // height
    constexpr int SCORE    = 4;  // 置信度概率 [0, 1]
    constexpr int KPT_BASE = 5;  // 关键点起始 (kpt_x0, kpt_y0, kpt_x1, ...)
    constexpr int FEATURE_SIZE = 17;  // 4 + 1 + 6*2
    constexpr int NUM_KPTS = 6;
}

std::vector<RawBuffObject> Sp25Decoder::decode(
    const float* data,
    const std::vector<int64_t>& shape,
    const LetterboxMeta& meta)
{
    // shape 应为 [1, C, N] 或 [1, N, C]
    if (shape.size() < 3) {
        return {};
    }

    int64_t dim1 = shape[1];
    int64_t dim2 = shape[2];

    // 自动检测布局:
    //   CHW: dim1 = 17 (特征), dim2 = 8400 (候选)
    //   NHW: dim1 = 8400 (候选), dim2 = 17 (特征)
    bool is_chw = (dim1 < dim2);  // dim1 < dim2 说明 dim1 是特征维
    int64_t n_candidates = is_chw ? dim2 : dim1;
    int64_t n_features   = is_chw ? dim1 : dim2;

    if (n_features != sp25::FEATURE_SIZE) {
        return {};
    }

    std::vector<RawBuffObject> candidates;
    std::vector<cv::Rect>      boxes_for_nms;  // NMSBoxes 需要整数 Rect
    std::vector<float>         scores_for_nms;

    // 小幅预留，避免常见场景频繁扩容；不盲目按 8400 全量分配
    const size_t reserve_hint = static_cast<size_t>(std::min<int64_t>(n_candidates, 1024));
    candidates.reserve(reserve_hint);
    boxes_for_nms.reserve(reserve_hint);
    scores_for_nms.reserve(reserve_hint);

    // 根据布局确定元素访问方式
    // CHW: data[feature_idx * N + candidate_idx]
    // NHW: data[candidate_idx * C + feature_idx]
    for (int64_t i = 0; i < n_candidates; ++i) {
        // sp25 导出 score 已经是概率值 [0,1]，不能再当 logit 处理
        float score;
        if (is_chw) {
            score = data[sp25::SCORE * n_candidates + i];
        } else {
            score = data[i * n_features + sp25::SCORE];
        }

        // 概率阈值过滤
        if (score < conf_threshold_) {
            continue;
        }

        // 读取 box (cx, cy, w, h) — 网络坐标系
        float cx, cy, bw, bh;
        if (is_chw) {
            cx = data[sp25::BOX_CX * n_candidates + i];
            cy = data[sp25::BOX_CY * n_candidates + i];
            bw = data[sp25::BOX_W  * n_candidates + i];
            bh = data[sp25::BOX_H  * n_candidates + i];
        } else {
            cx = data[i * n_features + sp25::BOX_CX];
            cy = data[i * n_features + sp25::BOX_CY];
            bw = data[i * n_features + sp25::BOX_W];
            bh = data[i * n_features + sp25::BOX_H];
        }

        // 将网络坐标框转为 (x1, y1, w, h) 后还原到原图
        cv::Rect2f net_box(cx - bw * 0.5f, cy - bh * 0.5f, bw, bh);
        cv::Rect2f orig_box = meta.restore(net_box);

        // 读取关键点并还原
        RawBuffObject obj;
        obj.score = score;
        obj.box   = orig_box;
        obj.kpt_count = sp25::NUM_KPTS;

        for (int k = 0; k < sp25::NUM_KPTS; ++k) {
            float kx, ky;
            if (is_chw) {
                kx = data[(sp25::KPT_BASE + k * 2)     * n_candidates + i];
                ky = data[(sp25::KPT_BASE + k * 2 + 1) * n_candidates + i];
            } else {
                kx = data[i * n_features + sp25::KPT_BASE + k * 2];
                ky = data[i * n_features + sp25::KPT_BASE + k * 2 + 1];
            }
            obj.kpts[k] = meta.restore(cv::Point2f(kx, ky));
        }

        candidates.push_back(obj);

        // NMS 需要整数 Rect (OpenCV NMSBoxes API)
        boxes_for_nms.push_back(cv::Rect(
            static_cast<int>(orig_box.x),
            static_cast<int>(orig_box.y),
            static_cast<int>(orig_box.width),
            static_cast<int>(orig_box.height)
        ));
        scores_for_nms.push_back(score);
    }

    if (candidates.empty()) {
        return {};
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes_for_nms, scores_for_nms,
                      conf_threshold_, nms_threshold_, indices);

    std::vector<RawBuffObject> results;
    results.reserve(indices.size());
    for (int idx : indices) {
        results.push_back(candidates[idx]);
    }

    return results;
}

}  // namespace autobuff::detector
