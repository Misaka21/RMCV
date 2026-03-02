#ifndef AIMER_AUTOBUFF_DETECTOR_COMMON_PREPROCESS_HPP
#define AIMER_AUTOBUFF_DETECTOR_COMMON_PREPROCESS_HPP

#include <utility>

#include <opencv2/core.hpp>

#include "raw_types.hpp"

namespace autobuff::detector {

/**
 * @brief BGR uint8 letterbox resize
 *
 * 等比例缩放 src 到 target_size x target_size 的正方形，
 * 多余部分用灰色 (114, 114, 114) 填充（居中对齐）。
 *
 * @param src         输入图像 (BGR, uint8)
 * @param target_size 输出边长
 * @return {letterbox 图像, LetterboxMeta}
 */
std::pair<cv::Mat, LetterboxMeta> letterbox_resize(
    const cv::Mat& src,
    int target_size = 640
);

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_COMMON_PREPROCESS_HPP
