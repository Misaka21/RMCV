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

#include "traditional/types.hpp"
#include "traditional/armor_detector.hpp"
#include "plugin/debug/logger.hpp"
#include "hardware/hardware_node.hpp"

namespace autoaim {

using TimePoint = std::chrono::steady_clock::time_point;

// 检测结果
struct DetectionResult {
    int frame_id = 0;
    TimePoint timestamp;
    std::vector<detector::Armor> armors;
    float detect_latency_ms = 0;
};

// Debug可视化
inline void draw_debug_visualization(const cv::Mat& image,
                                     const DetectionResult& result,
                                     const hardware::SyncFrame& frame,
                                     detector::Detector* detector) {
    cv::Mat debug_img = image.clone();
    detector->draw_results(debug_img);

    std::string info = fmt::format("Armors: {} Latency: {:.1f}ms",
        result.armors.size(), result.detect_latency_ms);
    cv::putText(debug_img, info, cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    if (frame.serial_valid) {
        auto imu = frame.imu();
        std::string imu_info = fmt::format("IMU: yaw={:.1f} pitch={:.1f}",
            imu.yaw, imu.pitch);
        cv::putText(debug_img, imu_info, cv::Point(10, 60),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
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

}  // namespace autoaim

#endif  // DETECTOR_NODE_HPP
