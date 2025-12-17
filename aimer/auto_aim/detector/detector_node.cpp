//
// Detector Node - 检测节点
// 订阅sync_frame，运行装甲板检测，发布检测结果
//

// C++ system headers
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

// Third-party headers
#include <fmt/format.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Project headers
#include "detector_node.hpp"
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "detector_factory.hpp"
#include "umt/umt.hpp"

namespace autoaim {

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

// Global detector instance
static std::unique_ptr<detector::Detector> g_detector = nullptr;
static std::atomic<detector::EnemyColor> g_detect_color{detector::EnemyColor::RED};

void set_enemy_color(detector::EnemyColor color) {
    g_detect_color = color;
    if (g_detector) {
        g_detector->detect_color = color;
    }
    debug::print(debug::PrintMode::INFO, "DetectorNode", "Enemy color set to {}",
                 color == detector::EnemyColor::RED ? "RED" : "BLUE");
}

void start_detector_node(detector::EnemyColor color) {
    debug::print(debug::PrintMode::INFO, "DetectorNode", "Starting detector node...");

    try {
        // 1. Create detector from config
        g_detect_color = color;
        g_detector = detector::create_detector_from_config(color, true);
        debug::print(debug::PrintMode::LOG, "DetectorNode", "Detector created from config");

        // 2. Setup UMT
        umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
        umt::Publisher<DetectionResult> pub("detection_result");
        auto running = umt::BasicObjManager<bool>::find_or_create("detector_running", true);
        auto debug_mode = umt::BasicObjManager<bool>::find_or_create("detector_debug", false);

        debug::print(debug::PrintMode::LOG, "DetectorNode", "Detector node started");

        // Stats
        int fps_count = 0;
        int detect_count = 0;
        float total_latency = 0;
        auto stats_time = SteadyClock::now();

        // 3. Main loop
        while (running->get()) {
            try {
                // Wait for frame with timeout
                auto frame = sub.pop_for(1000);  // 1 second timeout

                if (frame.image.empty()) {
                    continue;
                }

                // Update color if changed
                if (g_detector->detect_color != g_detect_color) {
                    g_detector->detect_color = g_detect_color;
                }

                // Run detection
                auto detect_start = SteadyClock::now();
                auto armors = g_detector->detect(frame.image);
                auto detect_end = SteadyClock::now();

                float latency = std::chrono::duration_cast<std::chrono::microseconds>(
                    detect_end - detect_start
                ).count() / 1000.0f;

                // Build result
                DetectionResult result;
                result.frame_id = frame.frame_id;
                result.timestamp = frame.timestamp;
                result.armors = std::move(armors);
                result.detect_latency_ms = latency;

                // Publish
                pub.push(result);

                // Stats
                fps_count++;
                if (!result.armors.empty()) {
                    detect_count++;
                }
                total_latency += latency;

                // Debug visualization
                if (debug_mode->get()) {
                    cv::Mat debug_img = frame.image.clone();
                    g_detector->draw_results(debug_img);

                    // Draw stats
                    std::string info = fmt::format("Armors: {} Latency: {:.1f}ms",
                        result.armors.size(), latency);
                    cv::putText(debug_img, info, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                    // Draw IMU info if valid
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

                // Print stats every second
                auto now = SteadyClock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_time).count() >= 1000) {
                    float avg_latency = fps_count > 0 ? total_latency / fps_count : 0;
                    debug::print(debug::PrintMode::DEBUG, "DetectorNode",
                        "FPS: {}, detected: {}, avg_latency: {:.1f}ms",
                        fps_count, detect_count, avg_latency);
                    fps_count = 0;
                    detect_count = 0;
                    total_latency = 0;
                    stats_time = now;
                }

            } catch (const umt::MessageError_Timeout&) {
                debug::print(debug::PrintMode::WARNING, "DetectorNode", "Timeout waiting for frame");
            }
        }

        // Cleanup
        if (debug_mode->get()) {
            cv::destroyWindow("Detector Debug");
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "DetectorNode", "Init failed: {}", e.what());
    }
}

}  // namespace autoaim
