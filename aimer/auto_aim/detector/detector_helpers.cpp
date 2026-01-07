//
// Detector Helpers - 实现
//

#include "detector_helpers.hpp"

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

namespace autoaim::detector {

cv::Mat draw_debug_overlay(
    const cv::Mat& image,
    const std::vector<DetectedArmor>& armors,
    float fps,
    float latency_ms
) {
    cv::Mat debug_img = image.clone();

    // 绘制装甲板
    for (const auto& armor : armors) {
        const auto& pts = armor.landmarks;
        if (pts.size() < 4) continue;

        // 绘制四边形轮廓
        for (size_t i = 0; i < pts.size(); i++) {
            cv::line(debug_img, pts[i], pts[(i + 1) % pts.size()],
                     cv::Scalar(0, 255, 0), 2);
        }

        // 绘制中心点
        cv::circle(debug_img, armor.center, 5, cv::Scalar(0, 0, 255), -1);

        // 标注数字
        cv::putText(debug_img, armor_number_to_string(armor.number),
                    armor.center + cv::Point2f(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
    }

    // 统计信息
    std::string info = fmt::format("FPS:{:.0f} Lat:{:.1f}ms Cnt:{}",
                                   fps, latency_ms, armors.size());
    cv::putText(debug_img, info, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    return debug_img;
}

}  // namespace autoaim::detector
