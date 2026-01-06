//
// Detector Node - 检测节点
//
// 根据检测器类型自动选择模式:
//   - is_async() = false: 单线程同步模式 (传统检测器)
//   - is_async() = true:  双线程异步模式 (YOLO检测器)
//

#include <atomic>
#include <memory>
#include <thread>

#include <fmt/format.h>

#include "detector_factory.hpp"
#include "detector_node.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"
#include "umt/umt.hpp"

namespace autoaim {

using SteadyClock = std::chrono::steady_clock;

// ============================================================================
// 同步模式 (传统检测器)
// ============================================================================

void run_sync_loop(detector::DetectorInterface* detector) {
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
            if (frame.image.empty()) continue;
            if (!frame.serial_valid) continue;

            // 更新颜色
            if (frame.serial_data.enemy_color != 0) {
                auto color = (frame.serial_data.enemy_color == 1)
                    ? detector::EnemyColor::RED
                    : detector::EnemyColor::BLUE;
                detector->set_enemy_color(color);
            }

            // 同步检测
            auto detect_start = SteadyClock::now();
            auto armors = detector->detect(frame.image);
            auto detect_end = SteadyClock::now();

            float latency_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                detect_end - detect_start).count() / 1000.0f;

            // 构建 DetectionResult
            aimer::DetectionResult result;
            result.frame_id = frame.frame_id;
            result.armors = std::move(armors);
            result.latency_ms = latency_ms;
            result.img = frame.image;  // 始终传递图片给 predictor

            const auto& s = frame.serial_data;
            result.state.set_euler(s.yaw, s.pitch, s.roll);
            result.state.bullet_speed = s.bullet_speed;
            result.state.enemy_color = s.enemy_color;
            result.state.aim_mode = s.aim_mode;
            result.state.allow_fire = s.allow_fire;
            result.state.timestamp_us = frame.timestamp_us;

            pub.push(result);
            stats.update(latency_ms, !result.armors.empty());

            // 更新 Dashboard 数据
            dashboard::set("detector.latency_ms", latency_ms);
            dashboard::set("detector.armor_count", static_cast<int>(result.armors.size()));
            dashboard::set("detector.fps", stats.last_fps);

            // 发布 Debug 图像给 Web (只在有订阅者时处理)
            if (pub_debug.has_subscriber()) {
                cv::Mat debug_img = frame.image.clone();
                for (const auto& armor : result.armors) {
                    const auto& pts = armor.landmarks;
                    for (size_t i = 0; i < pts.size(); i++) {
                        cv::line(debug_img, pts[i], pts[(i+1)%pts.size()],
                                 cv::Scalar(0, 255, 0), 2);
                    }
                    cv::circle(debug_img, armor.center, 5, cv::Scalar(0, 0, 255), -1);
                    cv::putText(debug_img, armor_number_to_string(armor.number),
                                armor.center + cv::Point2f(10, -10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
                }
                std::string info = fmt::format("FPS:{} Lat:{:.1f}ms Cnt:{}",
                    stats.last_fps, latency_ms, result.armors.size());
                cv::putText(debug_img, info, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                pub_debug.push(debug_img);
            }

            // 原始 debug 窗口 (包含世界坐标网格)
            if (debug_mode) {
                draw_debug_visualization(frame.image, result, frame);
            }

        } catch (const umt::MessageError_Timeout&) {
            // 超时，继续
        }
    }

    if (debug_mode) {
        cv::destroyWindow("Detector Debug");
    }
}

// ============================================================================
// 异步模式 (YOLO 检测器)
// ============================================================================

void run_async_loop(detector::DetectorInterface* detector) {
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
    umt::Publisher<aimer::DetectionResult> pub("detections");
    umt::Publisher<cv::Mat> pub_debug("/detector/debug");
    auto running = umt::BasicObjManager<bool>::find_or_create("detector_running", true);

    auto config = static_param::parse_file("detector.toml");
    bool debug_mode = static_param::get_param<bool>(config, "Detector.traditional", "debug");

    stats::FpsStats push_stats("DetectorNode-Push", "", 5000);
    stats::FpsStats pop_stats("DetectorNode", "detected", 5000);

    std::atomic<detector::EnemyColor> current_color{detector::EnemyColor::RED};

    // Push 线程
    std::thread push_thread([&]() {
        debug::print(debug::PrintMode::INFO, "DetectorNode", "Push thread started");

        while (running->get()) {
        watchdog::heartbeat("detector");
            try {
                auto frame = sub.pop_for(1000);
                if (frame.image.empty()) continue;
                if (!frame.serial_valid) continue;

                if (frame.serial_data.enemy_color != 0) {
                    auto color = (frame.serial_data.enemy_color == 1)
                        ? detector::EnemyColor::RED
                        : detector::EnemyColor::BLUE;
                    if (current_color.load() != color) {
                        current_color.store(color);
                        detector->set_enemy_color(color);
                    }
                }

                detector->push(frame.image, frame.frame_id, frame.timestamp_us, frame.serial_data);
                push_stats.update();

            } catch (const umt::MessageError_Timeout&) {
                // 超时，继续
            }
        }

        debug::print(debug::PrintMode::INFO, "DetectorNode", "Push thread stopped");
    });

    debug::print(debug::PrintMode::INFO, "DetectorNode", "Running in async mode");

    // Pop 主循环
    while (running->get()) {
        watchdog::heartbeat("detector");
        if (detector->queue_size() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto async_result = detector->pop();

        aimer::DetectionResult result;
        result.frame_id = async_result.frame_id;
        result.armors = std::move(async_result.armors);
        result.latency_ms = async_result.latency_ms;
        result.img = async_result.image;

        const auto& s = async_result.serial_data;
        result.state.set_euler(s.yaw, s.pitch, s.roll);
        result.state.bullet_speed = s.bullet_speed;
        result.state.enemy_color = s.enemy_color;
        result.state.aim_mode = s.aim_mode;
        result.state.allow_fire = s.allow_fire;
        result.state.timestamp_us = async_result.timestamp_us;

        pub.push(result);
        pop_stats.update(async_result.latency_ms, !result.armors.empty());

        // 更新 Dashboard 数据
        dashboard::set("detector.latency_ms", async_result.latency_ms);
        dashboard::set("detector.armor_count", static_cast<int>(result.armors.size()));
        dashboard::set("detector.fps", pop_stats.last_fps);

        // 发布 Debug 图像给 Web (只在有订阅者时处理)
        if (pub_debug.has_subscriber() && !async_result.image.empty()) {
            cv::Mat debug_img = async_result.image.clone();
            for (const auto& armor : result.armors) {
                const auto& pts = armor.landmarks;
                for (size_t i = 0; i < pts.size(); i++) {
                    cv::line(debug_img, pts[i], pts[(i+1)%pts.size()],
                             cv::Scalar(0, 255, 0), 2);
                }
                cv::circle(debug_img, armor.center, 5, cv::Scalar(0, 0, 255), -1);
                cv::putText(debug_img, armor_number_to_string(armor.number),
                            armor.center + cv::Point2f(10, -10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
            }
            std::string info = fmt::format("FPS:{} Lat:{:.1f}ms Cnt:{}",
                pop_stats.last_fps, async_result.latency_ms, result.armors.size());
            cv::putText(debug_img, info, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            pub_debug.push(debug_img);
        }

        // 原始 debug 窗口 (包含世界坐标网格)
        if (debug_mode && !async_result.image.empty()) {
            hardware::SyncFrame frame;
            frame.image = async_result.image;
            frame.frame_id = async_result.frame_id;
            frame.timestamp_us = async_result.timestamp_us;
            frame.serial_data = async_result.serial_data;
            frame.serial_valid = true;
            draw_debug_visualization(async_result.image, result, frame);
        }
    }

    if (push_thread.joinable()) {
        push_thread.join();
    }

    if (debug_mode) {
        cv::destroyWindow("Detector Debug");
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

} // namespace autoaim
