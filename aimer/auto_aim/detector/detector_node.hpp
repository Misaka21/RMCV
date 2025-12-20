//
// Detector Node - 检测节点
// 订阅sync_frame，运行装甲板检测，发布检测结果
//

#ifndef DETECTOR_NODE_HPP
#define DETECTOR_NODE_HPP

#include <chrono>
#include <vector>

#include <fmt/format.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "common/detector_interface.hpp"
#include "common/types.hpp"

namespace autoaim {

using TimePoint = std::chrono::steady_clock::time_point;

// 检测结果 (使用公共类型)
struct DetectionResult {
    int frame_id = 0;
    TimePoint timestamp;
    std::vector<detector::DetectedArmor> armors;
    float detect_latency_ms = 0;
};

// Debug可视化
inline void draw_debug_visualization(
    const cv::Mat& image,
    const DetectionResult& result,
    const hardware::SyncFrame& frame,
    const std::vector<detector::DetectedArmor>& armors
) {
    cv::Mat debug_img = image.clone();

    // 绘制装甲板
    for (const auto& armor : armors) {
        if (armor.landmarks.size() >= 4) {
            // 绘制轮廓
            for (size_t i = 0; i < armor.landmarks.size(); ++i) {
                cv::line(
                    debug_img,
                    armor.landmarks[i],
                    armor.landmarks[(i + 1) % armor.landmarks.size()],
                    cv::Scalar(0, 255, 0),
                    2,
                    cv::LINE_AA
                );
            }
            // 显示数字和置信度
            std::string text = fmt::format(
                "{} {:.2f}",
                armor.number.empty() ? "?" : armor.number,
                armor.confidence
            );
            cv::putText(
                debug_img,
                text,
                armor.center - cv::Point2f(20, 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 255),
                2
            );
        }
    }

    std::string info =
        fmt::format("Armors: {} Latency: {:.1f}ms", result.armors.size(), result.detect_latency_ms);
    cv::putText(
        debug_img,
        info,
        cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 0),
        2
    );

    if (frame.serial_valid) {
        auto imu = frame.imu();
        std::string imu_info = fmt::format("IMU: yaw={:.1f} pitch={:.1f}", imu.yaw, imu.pitch);
        cv::putText(
            debug_img,
            imu_info,
            cv::Point(10, 60),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 255),
            2
        );
    }

    cv::Mat display;
    cv::resize(debug_img, display, cv::Size(960, 720));
    cv::imshow("Detector Debug", display);
    cv::waitKey(1);
}

// 启动检测节点 (颜色从串口获取)
void start_detector_node();

// 运行时设置敌方颜色
void set_enemy_color(detector::EnemyColor color);

} // namespace autoaim

#endif // DETECTOR_NODE_HPP
