//
// 能量机关检测器节点实现
//

#include "detector_node.hpp"

#include <atomic>
#include <memory>
#include <thread>

#include <fmt/format.h>
#include <opencv2/core/mat.hpp>

#include "detector_factory.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"
#include "umt/umt.hpp"

#include "hardware/hardware_node.hpp"

namespace autobuff::detector {

using SteadyClock = std::chrono::steady_clock;

// ============================================================================
// 辅助函数
// ============================================================================

namespace {

// 串口颜色转换为检测器颜色
// 注: 新协议不含 enemy_color，此函数保留但颜色需要从配置获取
EnemyColor serial_to_enemy_color(uint8_t serial_color) {
    switch (serial_color) {
        case 1: return EnemyColor::RED;
        case 2: return EnemyColor::BLUE;
        default: return EnemyColor::UNKNOWN;
    }
}

// 更新 Dashboard 数据
void update_dashboard(float latency_ms, int target_count, float fps, DetectionStatus status) {
    dashboard::set("buff_detector.latency_ms", latency_ms);
    dashboard::set("buff_detector.target_count", target_count);
    dashboard::set("buff_detector.fps", fps);
    dashboard::set("buff_detector.status", static_cast<int>(status));
}

// 构建 RobotState
aimer::RobotState build_robot_state(const serial::SerialReceiveData& data, int64_t timestamp_us) {
    aimer::RobotState state;
    // 新协议: yaw/pitch/roll 已经是弧度
    state.set_euler_rad(data.yaw, data.pitch, data.roll);
    state.bullet_speed = data.bullet_speed;
    state.aim_mode = aimer::to_aim_mode(data.aim_mode);
    state.aiming_lock = data.aiming_lock;
    // enemy_color 和 allow_fire 从配置加载（不在协议中）
    state.enemy_color = data.enemy_color;
    state.allow_fire = data.allow_fire;
    state.timestamp_us = timestamp_us;
    return state;
}

}  // namespace

// ============================================================================
// BuffDetectorNode 实现
// ============================================================================

BuffDetectorNode::BuffDetectorNode(const BuffDetectorNodeConfig& config)
    : config_(config) {}

BuffDetectorNode::~BuffDetectorNode() {
    stop();
}

void BuffDetectorNode::start() {
    if (running_.load()) {
        return;
    }

    running_.store(true);
    detection_thread_ = std::thread(&BuffDetectorNode::detection_loop, this);
}

void BuffDetectorNode::stop() {
    running_.store(false);
    if (detection_thread_.joinable()) {
        detection_thread_.join();
    }
}

void BuffDetectorNode::detection_loop() {
    debug::print(debug::PrintMode::INFO, "BuffDetectorNode", "Starting...");

    try {
        // 创建检测器
        detector_ = create_detector_from_config(EnemyColor::RED, config_.config_file);
        debug::print(debug::PrintMode::INFO, "BuffDetectorNode", "Detector created");

        if (detector_->is_async()) {
            process_frame_async();
        } else {
            process_frame_sync();
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::FATAL, "BuffDetectorNode", "Init failed: {}", e.what());
    }

    debug::print(debug::PrintMode::INFO, "BuffDetectorNode", "Stopped");
}

// ============================================================================
// 同步模式
// ============================================================================

void BuffDetectorNode::process_frame_sync() {
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
    umt::Publisher<BuffDetectionResult> pub("buff_detections");
    umt::Publisher<cv::Mat> pub_debug("/buff_detector/debug");

    auto config = static_param::parse_file(config_.config_file);
    bool debug_mode = static_param::get_param<bool>(config, "Detector", "debug");

    stats::FpsStats stats("BuffDetectorNode", "detected");

    debug::print(debug::PrintMode::INFO, "BuffDetectorNode", "Running in sync mode");

    while (running_.load()) {
        watchdog::heartbeat("buff_detector");

        try {
            auto frame = sub.pop_for(1000);
            if (frame.image.empty() || !frame.serial_valid) {
                continue;
            }

            // 检查是否为能量机关模式
            auto aim_mode = aimer::to_aim_mode(frame.serial_data.aim_mode);
            if (aim_mode != aimer::AimMode::ENERGY_SMALL &&
                aim_mode != aimer::AimMode::ENERGY_LARGE) {
                continue;  // 不是能量机关模式，跳过
            }

            // 检测
            auto t0 = SteadyClock::now();
            double timestamp_sec = frame.timestamp_us / 1e6;
            auto result = detector_->detect(frame.image, timestamp_sec);
            auto t1 = SteadyClock::now();

            float latency_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
            result.latency_ms = latency_ms;
            result.frame_id = frame.frame_id;
            result.robot_state = build_robot_state(frame.serial_data, frame.timestamp_us);
            result.image = frame.image;

            // 发布结果
            pub.push(result);

            // 统计
            frame_count_++;
            total_latency_ms_ += latency_ms;
            stats.update(latency_ms, result.target_count > 0);
            update_dashboard(latency_ms, result.target_count, stats.last_fps, result.status);

            // 发布调试图像
            if (pub_debug.has_subscriber()) {
                cv::Mat debug_img = detector_->get_debug_image();
                if (!debug_img.empty()) {
                    pub_debug.push(debug_img);
                }
            }

        } catch (const umt::MessageError_Timeout&) {
            // 超时，继续
        }
    }
}

// ============================================================================
// 异步模式
// ============================================================================

void BuffDetectorNode::process_frame_async() {
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");
    umt::Publisher<BuffDetectionResult> pub("buff_detections");
    umt::Publisher<cv::Mat> pub_debug("/buff_detector/debug");

    auto config = static_param::parse_file(config_.config_file);
    bool debug_mode = static_param::get_param<bool>(config, "Detector", "debug");

    stats::FpsStats push_stats("BuffDetectorNode-Push", "");
    stats::FpsStats pop_stats("BuffDetectorNode", "detected");

    // Push 线程
    std::thread push_thread([&]() {
        debug::print(debug::PrintMode::INFO, "BuffDetectorNode", "Push thread started");

        while (running_.load()) {
            watchdog::heartbeat("buff_detector");

            try {
                auto frame = sub.pop_for(1000);
                if (frame.image.empty() || !frame.serial_valid) {
                    continue;
                }

                // 检查是否为能量机关模式
                auto aim_mode = aimer::to_aim_mode(frame.serial_data.aim_mode);
                if (aim_mode != aimer::AimMode::ENERGY_SMALL &&
                    aim_mode != aimer::AimMode::ENERGY_LARGE) {
                    continue;
                }

                detector_->push(frame.image, frame.frame_id,
                               frame.timestamp_us, frame.serial_data);
                push_stats.update();

            } catch (const umt::MessageError_Timeout&) {
                // 超时，继续
            }
        }

        debug::print(debug::PrintMode::INFO, "BuffDetectorNode", "Push thread stopped");
    });

    debug::print(debug::PrintMode::INFO, "BuffDetectorNode", "Running in async mode");

    // Pop 主循环
    while (running_.load()) {
        watchdog::heartbeat("buff_detector");

        if (detector_->queue_size() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto async_result = detector_->pop();

        // 补充结果信息
        async_result.detection.robot_state = build_robot_state(
            async_result.serial_data, async_result.timestamp_us);
        async_result.detection.image = async_result.image;
        async_result.detection.latency_ms = async_result.latency_ms;

        // 发布结果
        pub.push(async_result.detection);

        // 统计
        frame_count_++;
        total_latency_ms_ += async_result.latency_ms;
        pop_stats.update(async_result.latency_ms, async_result.detection.target_count > 0);
        update_dashboard(async_result.latency_ms, async_result.detection.target_count,
                        pop_stats.last_fps, async_result.detection.status);

        // 发布调试图像
        if (pub_debug.has_subscriber()) {
            cv::Mat debug_img = detector_->get_debug_image();
            if (!debug_img.empty()) {
                pub_debug.push(debug_img);
            }
        }
    }

    if (push_thread.joinable()) {
        push_thread.join();
    }
}

// ============================================================================
// 后台启动
// ============================================================================

void background_buff_detector_run(const std::string& config_file) {
    BuffDetectorNodeConfig config;
    config.config_file = config_file;

    auto node = std::make_unique<BuffDetectorNode>(config);
    node->start();

    // 阻塞直到停止
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);
    while (running->get()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    node->stop();
}

}  // namespace autobuff::detector
