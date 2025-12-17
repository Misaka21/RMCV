//
// Detector Node - 检测节点
// 订阅sync_frame，运行装甲板检测，发布检测结果
//

#include <atomic>
#include <memory>

#include "detector_node.hpp"
#include "detector_factory.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "umt/umt.hpp"

namespace autoaim {

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

        stats::FpsStats stats("DetectorNode", "detected");

        // 3. Main loop
        while (running->get()) {
            try {
                auto frame = sub.pop_for(1000);
                if (frame.image.empty()) {
                    continue;
                }

                // 更新颜色
                if (g_detector->detect_color != g_detect_color) {
                    g_detector->detect_color = g_detect_color;
                }

                // 运行检测
                auto detect_start = SteadyClock::now();
                auto armors = g_detector->detect(frame.image);
                auto detect_end = SteadyClock::now();
                float latency = std::chrono::duration_cast<std::chrono::microseconds>(
                    detect_end - detect_start).count() / 1000.0f;

                // 构建结果
                DetectionResult result;
                result.frame_id = frame.frame_id;
                result.timestamp = frame.timestamp;
                result.armors = std::move(armors);
                result.detect_latency_ms = latency;

                pub.push(result);

                // 更新统计
                stats.update(latency, !result.armors.empty());

                // Debug可视化
                if (debug_mode->get()) {
                    draw_debug_visualization(frame.image, result, frame, g_detector.get());
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
