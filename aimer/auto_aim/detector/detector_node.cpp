//
// Detector Node - 检测节点
//

#include "detector_node.hpp"

#include <atomic>
#include <memory>
#include <thread>

#include <fmt/format.h>
#include <opencv2/core/mat.hpp>

#include "detector_factory.hpp"
#include "detector_helpers.hpp"
#include "detector_visualizer.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"
#include "umt/umt.hpp"

namespace autoaim {

using SteadyClock = std::chrono::steady_clock;

// ============================================================================
// 公共辅助
// ============================================================================

namespace {

// 更新检测器颜色
void update_detector_color(detector::DetectorInterface* det, uint8_t serial_color) {
    if (serial_color != 0) {
        det->set_enemy_color(detector::serial_to_enemy_color(serial_color));
    }
}

// 更新 Dashboard 数据
void update_dashboard(float latency_ms, size_t armor_count, float fps) {
    dashboard::set("detector.latency_ms", latency_ms);
    dashboard::set("detector.armor_count", static_cast<int>(armor_count));
    dashboard::set("detector.fps", fps);
}

// 发布 Debug 图像 (如果有订阅者)
void publish_debug_image(
    umt::Publisher<cv::Mat>& pub,
    const cv::Mat& image,
    const std::vector<detector::DetectedArmor>& armors,
    float fps,
    float latency_ms
) {
    if (pub.has_subscriber() && !image.empty()) {
        pub.push(detector::draw_debug_overlay(image, armors, fps, latency_ms));
    }
}

}  // namespace

// ============================================================================
// 同步模式 (传统检测器)
// ============================================================================

void run_sync_loop(detector::DetectorInterface* det) {
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
    umt::Publisher<aimer::DetectionResult> pub("detections");
    umt::Publisher<cv::Mat> pub_debug("/detector/debug");
    auto running = umt::BasicObjManager<bool>::find_or_create("detector_running", true);

    auto config = static_param::parse_file("detector.toml");
    bool debug_mode = static_param::get_param<bool>(config, "Detector.traditional", "debug");

    stats::FpsStats stats("DetectorNode", "detected", 5000);

    debug::print(debug::PrintMode::INFO, "DetectorNode", "Running in sync mode");

    while (running->get()) {
        watchdog::heartbeat("detector");
        try {
            auto frame = sub.pop_for(1000);
            if (frame.image.empty() || !frame.serial_valid) continue;

            // 更新颜色
            update_detector_color(det, frame.serial_data.enemy_color);

            // 检测
            auto t0 = SteadyClock::now();
            auto armors = det->detect(frame.image);
            auto t1 = SteadyClock::now();
            float latency_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

            // 构建并发布结果
            auto result = detector::build_detection_result(
                std::move(armors), frame.image,
                frame.frame_id, frame.timestamp_us,
                frame.serial_data, latency_ms
            );
            pub.push(result);

            // 统计
            stats.update(latency_ms, !result.armors.empty());
            update_dashboard(latency_ms, result.armors.size(), stats.last_fps);

            // Web 调试图像
            publish_debug_image(pub_debug, frame.image, result.armors,
                                stats.last_fps, latency_ms);

            // 本地调试窗口
            if (debug_mode) {
                detector::draw_debug_visualization(frame.image, result, frame);
            }

        } catch (const umt::MessageError_Timeout&) {
            // 超时，继续
        }
    }

    if (debug_mode) {
        detector::close_debug_window();
    }
}

// ============================================================================
// 异步模式 (YOLO 检测器)
// ============================================================================

void run_async_loop(detector::DetectorInterface* det) {
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
    umt::Publisher<aimer::DetectionResult> pub("detections");
    umt::Publisher<cv::Mat> pub_debug("/detector/debug");
    auto running = umt::BasicObjManager<bool>::find_or_create("detector_running", true);

    auto config = static_param::parse_file("detector.toml");
    bool debug_mode = static_param::get_param<bool>(config, "Detector.traditional", "debug");

    stats::FpsStats push_stats("DetectorNode-Push", "", 5000);
    stats::FpsStats pop_stats("DetectorNode", "detected", 5000);

    std::atomic<detector::EnemyColor> current_color{detector::EnemyColor::RED};

    // Push 线程: 从相机读取帧，推送给检测器
    std::thread push_thread([&]() {
        debug::print(debug::PrintMode::INFO, "DetectorNode", "Push thread started");

        while (running->get()) {
            watchdog::heartbeat("detector");
            try {
                auto frame = sub.pop_for(1000);
                if (frame.image.empty() || !frame.serial_valid) continue;

                // 更新颜色
                if (frame.serial_data.enemy_color != 0) {
                    auto color = detector::serial_to_enemy_color(frame.serial_data.enemy_color);
                    if (current_color.load() != color) {
                        current_color.store(color);
                        det->set_enemy_color(color);
                    }
                }

                det->push(frame.image, frame.frame_id, frame.timestamp_us, frame.serial_data);
                push_stats.update();

            } catch (const umt::MessageError_Timeout&) {
                // 超时，继续
            }
        }

        debug::print(debug::PrintMode::INFO, "DetectorNode", "Push thread stopped");
    });

    debug::print(debug::PrintMode::INFO, "DetectorNode", "Running in async mode");

    // Pop 主循环: 获取检测结果并发布
    while (running->get()) {
        watchdog::heartbeat("detector");

        if (det->queue_size() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto async_result = det->pop();

        // 构建并发布结果
        auto result = detector::build_detection_result(async_result);
        pub.push(result);

        // 统计
        pop_stats.update(async_result.latency_ms, !result.armors.empty());
        update_dashboard(async_result.latency_ms, result.armors.size(), pop_stats.last_fps);

        // Web 调试图像
        publish_debug_image(pub_debug, async_result.image, result.armors,
                            pop_stats.last_fps, async_result.latency_ms);

        // 本地调试窗口
        if (debug_mode && !async_result.image.empty()) {
            hardware::SyncFrame frame;
            frame.image = async_result.image;
            frame.frame_id = async_result.frame_id;
            frame.timestamp_us = async_result.timestamp_us;
            frame.serial_data = async_result.serial_data;
            frame.serial_valid = true;
            detector::draw_debug_visualization(async_result.image, result, frame);
        }
    }

    if (push_thread.joinable()) {
        push_thread.join();
    }

    if (debug_mode) {
        detector::close_debug_window();
    }
}

// ============================================================================
// 入口
// ============================================================================

void start_detector_node() {
    debug::print(debug::PrintMode::INFO, "DetectorNode", "Starting detector node...");

    try {
        auto detector = detector::create_detector_from_config(detector::EnemyColor::RED);
        debug::print(debug::PrintMode::INFO, "DetectorNode", "Detector created");

        if (detector->is_async()) {
            run_async_loop(detector.get());
        } else {
            run_sync_loop(detector.get());
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::FATAL, "DetectorNode", "Init failed: {}", e.what());
        std::exit(1);
    }
}

}  // namespace autoaim
