//
// Detector Node - 检测节点
// 订阅sync_frame，运行装甲板检测，发布检测结果
//

#include <atomic>
#include <memory>

#include <fmt/format.h>

#include "detector_node.hpp"
#include "detector_factory.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/webview/dashboard_data.hpp"
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
        g_detector = detector::create_detector_from_config(color);
        debug::print(debug::PrintMode::LOG, "DetectorNode", "Detector created from config");

        // 2. Setup UMT
        umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
        umt::Publisher<DetectionResult> pub("detection_result");
        auto running = umt::BasicObjManager<bool>::find_or_create("detector_running", true);

        // Dashboard 数据 (通过 UMT ObjManager 共享到 Python)
        auto dashboard = umt::ObjManager<DashboardData>::find_or_create("dashboard");

        // 图像话题发布器 (只在 debug 模式使用)
        umt::Publisher<cv::Mat> pub_debug("/detector/debug");

        // 从配置文件读取 debug 模式
        auto config = static_param::parse_file("detector.toml");
        bool debug_mode = static_param::get_param<bool>(config, "Detector.traditional", "debug");

        debug::print(debug::PrintMode::LOG, "DetectorNode", "Detector node started");

        stats::FpsStats stats("DetectorNode", "detected");

        // 3. Main loop
        while (running->get()) {
            try {
                auto frame = sub.pop_for(1000);
                if (frame.image.empty()) {
                    continue;
                }

                // 更新颜色：优先使用串口传来的颜色，否则用全局设置
                detector::EnemyColor current_color = g_detect_color;
                if (frame.serial_valid && frame.serial_data.enemy_color != 0) {
                    // 串口颜色: 1=红, 2=蓝
                    current_color = (frame.serial_data.enemy_color == 1)
                        ? detector::EnemyColor::RED
                        : detector::EnemyColor::BLUE;
                }
                if (g_detector->detect_color != current_color) {
                    g_detector->detect_color = current_color;
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

                // 更新 Dashboard 数据 (供 Python Web 读取)
                {
                    dashboard->detect_latency_ms = latency;
                    dashboard->armor_count = static_cast<int>(result.armors.size());
                    dashboard->enemy_color = (current_color == detector::EnemyColor::RED) ? "RED" : "BLUE";
                    dashboard->detector_fps = stats.last_fps;

                    // 目标信息
                    if (!result.armors.empty()) {
                        const auto& target = result.armors[0];
                        dashboard->target_number = target.number;
                        dashboard->target_x = target.center.x;
                        dashboard->target_y = target.center.y;
                    }

                    // 串口数据
                    if (frame.serial_valid) {
                        dashboard->imu_yaw = frame.serial_data.yaw;
                        dashboard->imu_pitch = frame.serial_data.pitch;
                        dashboard->imu_roll = frame.serial_data.roll;
                        dashboard->bullet_speed = frame.serial_data.bullet_speed;
                        dashboard->aim_mode = frame.serial_data.aim_mode;
                        dashboard->allow_fire = frame.serial_data.allow_fire;
                        dashboard->serial_valid = true;
                    } else {
                        dashboard->serial_valid = false;
                    }
                }

                // Debug可视化 (只在有订阅者时处理，不阻塞主循环)
                if (debug_mode && pub_debug.has_subscriber()) {
                    // 直接在原图上画，避免 clone 开销
                    for (const auto& armor : result.armors) {
                        auto pts = armor.landmarks();
                        for (size_t i = 0; i < pts.size(); i++) {
                            cv::line(frame.image, pts[i], pts[(i+1)%pts.size()],
                                     cv::Scalar(0, 255, 0), 2);
                        }
                        cv::circle(frame.image, armor.center, 5, cv::Scalar(0, 0, 255), -1);
                        cv::putText(frame.image, armor.number, armor.center + cv::Point2f(10, -10),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
                    }
                    std::string info = fmt::format("FPS:{} Lat:{:.1f}ms Cnt:{}",
                        stats.last_fps, latency, result.armors.size());
                    cv::putText(frame.image, info, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                    pub_debug.push(frame.image);
                }

            } catch (const umt::MessageError_Timeout&) {
                debug::print(debug::PrintMode::WARNING, "DetectorNode", "Timeout waiting for frame");
            }
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "DetectorNode", "Init failed: {}", e.what());
    }
}

}  // namespace autoaim
