#ifndef AIMER_AUTOBUFF_DETECTOR_COMMON_RAW_TYPES_HPP
#define AIMER_AUTOBUFF_DETECTOR_COMMON_RAW_TYPES_HPP

#include <array>
#include <vector>
#include <opencv2/core.hpp>

#include "aimer/auto_buff/common/types.hpp"

namespace autobuff::detector {

// Letterbox 变换元数据，用于将网络坐标还原为原图坐标
struct LetterboxMeta {
    int src_w = 0, src_h = 0;
    int net_w = 480, net_h = 480;
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

// 能量机关类型 (YOLOX 模型输出)
enum class RuneType : uint8_t {
    INACTIVATED = 0,   // 未激活 (当前目标，需要打击)
    ACTIVATED   = 1,   // 已激活 (已被打击)
};

// 单个能量机关臂的原始检测结果 (YOLOX 模型输出)
//
// 模型: FYT yolox_rune_3.6m, 输入 480x480
// 每个检测包含 5 个关键点:
//   kpt[0] = r_center     (R 标中心)
//   kpt[1] = bottom_left  (靠近R标, 左侧)
//   kpt[2] = top_left     (远离R标, 左侧)
//   kpt[3] = top_right    (远离R标, 右侧)
//   kpt[4] = bottom_right (靠近R标, 右侧)
//
// 颜色: 由于训练时标签反转, 网络输出 class0=BLUE, class1=RED
struct RawRuneObject {
    cv::Point2f r_center{};                      // R 标中心 (kpt[0])
    std::array<cv::Point2f, 4> armor_corners{};  // 装甲板四角 (kpt[1-4])
    autobuff::EnemyColor color = autobuff::EnemyColor::UNKNOWN;
    RuneType type = RuneType::INACTIVATED;
    float score = 0.f;
    cv::Rect box{};
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_COMMON_RAW_TYPES_HPP
