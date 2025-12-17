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
using SteadyClock = std::chrono::steady_clock;

// 检测结果
struct DetectionResult {
    int frame_id = 0;
    TimePoint timestamp;
    std::vector<detector::Armor> armors;
    float detect_latency_ms = 0;
};

// 检测器统计信息
struct DetectorStats {
    int fps_count = 0;
    int detect_count = 0;
    float total_latency = 0;
    SteadyClock::time_point last_print_time = SteadyClock::now();

    void update(float latency, bool detected) {
        fps_count++;
        total_latency += latency;
        if (detected) {
            detect_count++;
        }
    }

    void print_if_needed() {
        auto now = SteadyClock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time).count();
        if (elapsed >= 1000) {
            float avg_latency = fps_count > 0 ? total_latency / fps_count : 0;
            debug::print(debug::PrintMode::DEBUG, "DetectorNode",
                "FPS: {}, detected: {}, avg_latency: {:.1f}ms",
                fps_count, detect_count, avg_latency);
            fps_count = 0;
            detect_count = 0;
            total_latency = 0;
            last_print_time = now;
        }
    }
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

    if (frame.imu_valid) {
        std::string imu_info = fmt::format("IMU: yaw={:.1f} pitch={:.1f}",
            frame.imu.yaw, frame.imu.pitch);
        cv::putText(debug_img, imu_info, cv::Point(10, 60),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }

    cv::Mat display;
    cv::resize(debug_img, display, cv::Size(960, 720));
    cv::imshow("Detector Debug", display);
    cv::waitKey(1);
}

// 启动检测节点
void start_detector_node(detector::EnemyColor color);

// 运行时设置敌方颜色
void set_enemy_color(detector::EnemyColor color);

}  // namespace autoaim

#endif  // DETECTOR_NODE_HPP
