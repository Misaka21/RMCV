#ifndef AIMER_AUTOBUFF_DETECTOR_COMMON_RAW_TYPES_HPP
#define AIMER_AUTOBUFF_DETECTOR_COMMON_RAW_TYPES_HPP

#include <array>
#include <opencv2/core.hpp>

namespace autobuff::detector {

// Letterbox 变换元数据，用于将网络坐标还原为原图坐标
struct LetterboxMeta {
    int src_w = 0, src_h = 0;
    int net_w = 640, net_h = 640;
    float scale = 1.f;
    float pad_x = 0.f, pad_y = 0.f;

    // 将网络坐标点还原到原图坐标
    cv::Point2f restore(const cv::Point2f& pt) const {
        return {(pt.x - pad_x) / scale, (pt.y - pad_y) / scale};
    }

    // 将网络坐标框还原到原图坐标
    cv::Rect2f restore(const cv::Rect2f& box) const {
        return {(box.x - pad_x) / scale, (box.y - pad_y) / scale,
                box.width / scale, box.height / scale};
    }
};

// 单个扇叶的原始检测结果 (解码器输出)
// sp25 模型输出: [1, 17, 8400]
//   4 box + 1 score + 6*2 keypoints = 17
//   kpt[0-3]: 扇叶四角 (左上逆时针)
//   kpt[4]:   扇叶中心
//   kpt[5]:   内侧尖端 (指向R标方向)
struct RawBuffObject {
    cv::Rect2f box{};       // 还原到原图的包围盒
    float score = 0.f;      // 置信度 (已过sigmoid)
    std::array<cv::Point2f, 6> kpts{};  // 还原到原图的关键点
    uint8_t kpt_count = 0;  // 有效关键点数 (sp25: 6)
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_COMMON_RAW_TYPES_HPP
