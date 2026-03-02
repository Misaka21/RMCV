#ifndef AIMER_AUTOBUFF_DETECTOR_DECODER_BUFF_DECODER_HPP
#define AIMER_AUTOBUFF_DETECTOR_DECODER_BUFF_DECODER_HPP

#include <cstdint>
#include <memory>
#include <vector>

#include "aimer/auto_buff/detector/common/raw_types.hpp"

namespace autobuff::detector {

/**
 * @brief 检测解码器抽象接口
 *
 * 从模型原始输出张量解码为 RawBuffObject 列表。
 * 子类负责特定输出格式的解析（置信度过滤、NMS、坐标还原）。
 */
class IBuffDecoder {
public:
    virtual ~IBuffDecoder() = default;

    /**
     * @brief 解码模型输出
     * @param data    输出张量数据指针 (float32)
     * @param shape   张量形状 [batch, C, N] 或 [batch, N, C]
     * @param meta    Letterbox 变换参数，用于坐标还原
     * @return 过滤后的检测列表 (已应用 NMS)
     */
    virtual std::vector<RawBuffObject> decode(
        const float* data,
        const std::vector<int64_t>& shape,
        const LetterboxMeta& meta
    ) = 0;
};

/**
 * @brief sp25 模型解码器
 *
 * 支持 sp25 单类别扇叶检测模型：
 *   输出格式: [1, 17, N] 或 [1, N, 17]
 *   其中 17 = 4 box + 1 score + 6*2 keypoints
 *   N = 8400 (640x640 输入时)
 *
 * 自动检测 CHW vs NHW 布局：
 *   如果 dim[1] < dim[2]，则 C=dim[1], N=dim[2]  (CHW, 需要转置访问)
 *   否则 N=dim[1], C=dim[2]                       (NHW, 行优先访问)
 *
 * 单类别，无类别 logits，直接用置信度过滤。
 */
class Sp25Decoder : public IBuffDecoder {
public:
    explicit Sp25Decoder(float conf_threshold = 0.45f, float nms_threshold = 0.45f)
        : conf_threshold_(conf_threshold), nms_threshold_(nms_threshold) {}

    std::vector<RawBuffObject> decode(
        const float* data,
        const std::vector<int64_t>& shape,
        const LetterboxMeta& meta
    ) override;

private:
    float conf_threshold_;
    float nms_threshold_;

    static float sigmoid(float x);
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_DECODER_BUFF_DECODER_HPP
