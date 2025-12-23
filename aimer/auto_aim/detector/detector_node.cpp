//
// Detector Node - 检测节点
// 订阅sync_frame，运行装甲板检测，发布检测结果
//

#include <memory>

#include "detector_factory.hpp"
#include "detector_node.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "umt/umt.hpp"

namespace autoaim {

using SteadyClock = std::chrono::steady_clock;

void start_detector_node() {
    std::unique_ptr<detector::DetectorInterface> g_detector = nullptr;
    debug::print(debug::PrintMode::INFO, "DetectorNode", "Starting detector node...");

    try {
        // 1. Create detector (初始颜色会从串口获取)
        g_detector = detector::create_detector_from_config(detector::EnemyColor::RED);
        debug::print(debug::PrintMode::INFO, "DetectorNode", "Detector created from config");

        // 2. Setup UMT
        umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
        umt::Publisher<aimer::DetectionResult> pub("detections");
        auto running = umt::BasicObjManager<bool>::find_or_create("detector_running", true);

        // 从配置文件读取 debug 模式
        auto config = static_param::parse_file("detector.toml");
        bool debug_mode = static_param::get_param<bool>(config, "Detector.traditional", "debug");

        debug::print(debug::PrintMode::INFO, "DetectorNode", "Detector node started");

        stats::FpsStats stats("DetectorNode", "detected");

        // 3. Main loop
        while (running->get()) {
            try {
                auto frame = sub.pop_for(1000);
                if (frame.image.empty()) {
                    continue;
                }

                // 必须有有效串口数据才处理
                if (!frame.serial_valid) {
                    debug::print(
                        debug::PrintMode::WARNING,
                        "DetectorNode",
                        "No valid serial data, skipping frame"
                    );
                    continue;
                }

                // 颜色必须从串口获取
                if (frame.serial_data.enemy_color == 0) {
                    frame.serial_data.enemy_color = 1;
                    //continue;
                }
                detector::EnemyColor current_color = (frame.serial_data.enemy_color == 1)
                    ? detector::EnemyColor::RED
                    : detector::EnemyColor::BLUE;

                if (g_detector->get_enemy_color() != current_color) {
                    g_detector->set_enemy_color(current_color);
                }

                // 运行检测
                auto detect_start = SteadyClock::now();
                auto armors = g_detector->detect(frame.image);
                auto detect_end = SteadyClock::now();
                float latency =
                    std::chrono::duration_cast<std::chrono::microseconds>(detect_end - detect_start)
                        .count()
                    / 1000.0f;

                // 构建结果
                aimer::DetectionResult result;
                result.frame_id = frame.frame_id;
                result.state = aimer::RobotState::from_sync_frame(frame);
                result.armors = std::move(armors);
                result.latency_ms = latency;
                result.img = frame.image;  // 传递图像给后续节点

                pub.push(result);

                // 更新统计
                stats.update(latency, !result.armors.empty());

                // Debug可视化
                if (debug_mode) {
                    draw_debug_visualization(frame.image, result, frame);
                }

            } catch (const umt::MessageError_Timeout&) {
                debug::print(
                    debug::PrintMode::WARNING,
                    "DetectorNode",
                    "Timeout waiting for frame"
                );
            }
        }

        // Cleanup
        if (debug_mode) {
            cv::destroyWindow("Detector Debug");
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::FATAL, "DetectorNode", "Init failed: {}", e.what());
        std::exit(1);
    }
}

} // namespace autoaim
