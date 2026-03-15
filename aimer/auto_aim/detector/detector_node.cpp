//
// Detector Node - 检测节点
//

#include "detector_node.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <fmt/format.h>
#include <opencv2/core/mat.hpp>

#include "detector_factory.hpp"
#include "detector_helpers.hpp"
#include "aimer/common/robot_state.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/rerun/rmcv_rerun.hpp"
#include "umt/umt.hpp"

namespace autoaim {

using SteadyClock = std::chrono::steady_clock;

// ============================================================================
// 公共辅助
// ============================================================================

namespace {

bool get_runtime_bool_or(const std::string& name, bool default_value) {
    auto ptr = runtime_param::find_param(name);
    if (ptr == nullptr) return default_value;
    if (auto* val = std::get_if<bool>(&*ptr)) return *val;
    return default_value;
}

std::string get_runtime_string_or(const std::string& name, std::string default_value) {
    auto ptr = runtime_param::find_param(name);
    if (ptr == nullptr) return default_value;
    if (auto* val = std::get_if<std::string>(&*ptr)) return *val;
    return default_value;
}

// 更新 Dashboard 数据
void update_dashboard(float latency_ms, size_t armor_count, float fps) {
    rr::scalar("detector/latency_ms", static_cast<double>(latency_ms));
    rr::scalar("detector/armor_count", static_cast<int>(armor_count));
    rr::scalar("detector/fps", static_cast<double>(fps));
}

bool should_generate_overlay(umt::Publisher<cv::Mat>& pub_debug, bool debug_mode) {
    if (pub_debug.has_subscriber() || debug_mode) return true;

    // visualizer 的 detector 视图通过 detector_debug_img 对象读取，不走消息订阅。
    // 这里在 detector 节点按需检查视图状态，避免无意义地每帧绘制 overlay。
    bool show_window = get_runtime_bool_or("Visualizer.show_window", false);
    if (!show_window) return false;
    return get_runtime_string_or("Visualizer.view", "") == "detector";
}

}  // namespace

// ============================================================================
// 同步模式 (传统检测器)
// ============================================================================

void run_sync_loop(detector::DetectorInterface* det) {
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
    umt::Publisher<DetectionResult> pub("detections");
    umt::Publisher<cv::Mat> pub_debug("/detector/debug");
    auto detector_debug_img = umt::BasicObjManager<cv::Mat>::find_or_create("detector_debug_img");
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    auto config = static_param::parse_file("armor_detector.toml");
    bool debug_mode = static_param::get_param<bool>(config, "Detector", "debug");

    stats::FpsStats stats("DetectorNode", "detected");

    debug::print(debug::PrintMode::INFO, "DetectorNode", "Running in sync mode");

    while (running->get()) {
        watchdog::heartbeat("detector");
        try {
            auto frame = sub.pop_for(1000);
            if (frame.image.empty() || !frame.serial_valid) continue;

            // 非自瞄模式时跳过检测
            auto aim_mode = aimer::to_aim_mode(frame.serial_data.aim_mode);
            if (aim_mode != aimer::AimMode::AUTOAIM) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // 敌方颜色（来自串口协议；0=未知则保持 detector 当前值）
            if (frame.serial_data.enemy_color == 1) {
                det->set_enemy_color(detector::EnemyColor::RED);
            } else if (frame.serial_data.enemy_color == 2) {
                det->set_enemy_color(detector::EnemyColor::BLUE);
            }

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

            // 调试图像数据流:
            // 1) 发布到 /detector/debug (web 调试/订阅方)
            // 2) 写入 detector_debug_img (visualizer detector 视图)
            // 3) 发送到 Rerun (跳帧 + 缩放)
            if (should_generate_overlay(pub_debug, debug_mode) && !frame.image.empty()) {
                cv::Mat overlay = detector::draw_debug_overlay(
                    frame.image, result.armors, stats.last_fps, latency_ms);
                detector_debug_img->get() = overlay;
                if (pub_debug.has_subscriber()) {
                    pub_debug.push(overlay);
                }
                rr::image("detector/preview", overlay, 2);
            }

        } catch (const umt::MessageError_Timeout&) {
            // 超时，继续
        } catch (const umt::MessageError_Stopped&) {
            break;
        }
    }
}

// ============================================================================
// 异步模式 (YOLO 检测器)
// ============================================================================

void run_async_loop(detector::DetectorInterface* det) {
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
    umt::Publisher<DetectionResult> pub("detections");
    umt::Publisher<cv::Mat> pub_debug("/detector/debug");
    auto detector_debug_img = umt::BasicObjManager<cv::Mat>::find_or_create("detector_debug_img");
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    auto config = static_param::parse_file("armor_detector.toml");
    bool debug_mode = static_param::get_param<bool>(config, "Detector", "debug");

    stats::FpsStats push_stats("DetectorNode-Push", "");
    stats::FpsStats pop_stats("DetectorNode", "detected");

    auto stop_async_detector = [&]() {
        det->stop();
    };

    // Push 线程: 从相机读取帧，推送给检测器
    std::thread push_thread([&]() {
        debug::print(debug::PrintMode::INFO, "DetectorNode", "Push thread started");

        while (running->get()) {
            watchdog::heartbeat("detector");
            try {
                auto frame = sub.pop_for(1000);
                if (frame.image.empty() || !frame.serial_valid) continue;

                // 非自瞄模式时跳过检测
                auto aim_mode = aimer::to_aim_mode(frame.serial_data.aim_mode);
                if (aim_mode != aimer::AimMode::AUTOAIM) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }

                // 敌方颜色（来自串口协议；0=未知则保持 detector 当前值）
                if (frame.serial_data.enemy_color == 1) {
                    det->set_enemy_color(detector::EnemyColor::RED);
                } else if (frame.serial_data.enemy_color == 2) {
                    det->set_enemy_color(detector::EnemyColor::BLUE);
                }

                det->push(frame.image, frame.frame_id, frame.timestamp_us, frame.serial_data);
                push_stats.update();

            } catch (const umt::MessageError_Timeout&) {
                // 超时，继续
            } catch (const umt::MessageError_Stopped&) {
                // Publisher 已销毁
                break;
            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::ERROR, "DetectorNode",
                             "Push loop exception: {}", e.what());
            } catch (...) {
                debug::print(debug::PrintMode::ERROR, "DetectorNode",
                             "Push loop unknown exception");
            }
        }

        debug::print(debug::PrintMode::INFO, "DetectorNode", "Push thread stopped");

        // 通知 pop 线程退出阻塞
        stop_async_detector();
    });

    debug::print(debug::PrintMode::INFO, "DetectorNode", "Running in async mode");

    // Pop 主循环: 获取检测结果并发布
    try {
        while (running->get()) {
            watchdog::heartbeat("detector");

            try {
                auto async_result = det->pop();  // 内部 condition_variable 阻塞等待
                if (async_result.image.empty()) continue;  // stop() 唤醒时返回空

                // 构建并发布结果
                auto result = detector::build_detection_result(async_result);
                pub.push(result);

                // 统计
                pop_stats.update(async_result.latency_ms, !result.armors.empty());
                update_dashboard(async_result.latency_ms, result.armors.size(), pop_stats.last_fps);

                // 调试图像数据流:
                // 1) 发布到 /detector/debug (web 调试/订阅方)
                // 2) 写入 detector_debug_img (visualizer detector 视图)
                // 3) 发送到 Rerun (跳帧 + 缩放)
                if (should_generate_overlay(pub_debug, debug_mode) && !async_result.image.empty()) {
                    cv::Mat overlay = detector::draw_debug_overlay(
                        async_result.image, result.armors, pop_stats.last_fps,
                        async_result.latency_ms);
                    detector_debug_img->get() = overlay;
                    if (pub_debug.has_subscriber()) {
                        pub_debug.push(overlay);
                    }
                    rr::image("detector/preview", overlay, 2);
                }

            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::ERROR, "DetectorNode",
                             "Pop loop exception: {}", e.what());
                running->set(false);
                break;
            } catch (...) {
                debug::print(debug::PrintMode::ERROR, "DetectorNode",
                             "Pop loop unknown exception");
                running->set(false);
                break;
            }
        }
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "DetectorNode",
                     "Async loop fatal exception: {}", e.what());
        running->set(false);
    } catch (...) {
        debug::print(debug::PrintMode::ERROR, "DetectorNode",
                     "Async loop fatal unknown exception");
        running->set(false);
    }

    // 统一退出路径：先唤醒阻塞，再回收线程，避免 joinable thread 触发 terminate。
    stop_async_detector();
    if (push_thread.joinable()) {
        push_thread.join();
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
