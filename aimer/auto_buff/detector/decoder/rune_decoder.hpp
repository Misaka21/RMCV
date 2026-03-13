// YOLOX 能量机关模型解码器
//
// 模型输出: [1, N_anchors, 15]
//   每个 anchor: 5*2 关键点坐标 + 1 objectness + 2 颜色得分 + 2 类型得分
//   strides: [8, 16, 32], 480x480 → 4725 anchors

#ifndef AIMER_AUTOBUFF_DETECTOR_DECODER_RUNE_DECODER_HPP
#define AIMER_AUTOBUFF_DETECTOR_DECODER_RUNE_DECODER_HPP

#include <vector>
#include <cstdint>

#include "aimer/auto_buff/detector/common/raw_types.hpp"

namespace autobuff::detector {

struct GridAndStride {
    int grid0;
    int grid1;
    int stride;
};

/**
 * @brief YOLOX 能量机关检测模型解码器
 *
 * 解码 FYT yolox_rune_3.6m 的输出:
 * - Grid + stride 基础解码 5 个关键点
 * - 颜色分类 (注意训练时标签反转)
 * - 类型分类 (INACTIVATED / ACTIVATED)
 * - NMS with merge (高IoU同类检测取平均)
 */
class RuneDecoder {
public:
    RuneDecoder(float conf_threshold = 0.50f,
                float nms_threshold = 0.30f,
                int top_k = 128,
                int input_w = 480,
                int input_h = 480);

    /**
     * @brief 解码模型输出
     * @param output_data  模型输出数据指针 (float32)
     * @param output_shape 输出张量形状 (e.g. [1, 4725, 15])
     * @param meta         Letterbox 元数据 (用于坐标还原)
     * @return 原图坐标下的检测结果列表
     */
    std::vector<RawRuneObject> decode(
        const float* output_data,
        const std::vector<int64_t>& output_shape,
        const LetterboxMeta& meta) const;

private:
    float conf_threshold_;
    float nms_threshold_;
    int top_k_;
    int input_w_;
    int input_h_;

    std::vector<GridAndStride> grid_strides_;

    void generate_grids_and_strides();
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_DECODER_RUNE_DECODER_HPP
