//
// Hardware Node - 硬件节点管理器
// 负责启动串口、相机采集和数据同步打包
//

// C++ system headers
#include <chrono>
#include <cmath>
#include <deque>
#include <optional>
#include <thread>

// Third-party library headers
#include <fmt/format.h>
#include <opencv2/core/mat.hpp>

// Project headers
#include "hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/webview/dashboard.hpp"
#include "hik_cam/hik_camera.hpp"
#include "serial/serial_thread.hpp"
#include "umt/umt.hpp"

namespace hardware {

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

// 带时间戳的串口数据，用于时间同步匹配
struct TimestampedSerialData {
    TimePoint recv_time;  // 接收时间
    serial::SerialReceiveData data;
};

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Load camera configuration from TOML
 */
camera::CameraConfig load_camera_config(const toml::table& config) {
    camera::CameraConfig cam_config;

    // Device selection
    cam_config.use_camera_sn = static_param::get_param<bool>(config, "Camera", "use_camera_sn");
    cam_config.camera_sn = static_param::get_param<std::string>(config, "Camera", "camera_sn");

    // MFS config file
    cam_config.use_mfs_config = static_param::get_param<bool>(config, "Camera", "use_config_from_file");
    std::string mfs_filename = static_param::get_param<std::string>(config, "Camera", "config_file_path");
    cam_config.mfs_config_path = std::string(CONFIG_DIR) + "/" + mfs_filename;

    // Runtime parameters
    cam_config.use_runtime_config = static_param::get_param<bool>(config, "Camera", "use_camera_config");

    // Get Camera.config table and convert to CameraParam
    auto param_table = static_param::get_param_table(config, "Camera.config");
    for (const auto& [key, value] : param_table) {
        std::visit([&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            // Skip vector types (not supported by camera API)
            if constexpr (!std::is_same_v<T, std::vector<int64_t>>) {
                cam_config.runtime_params.emplace_back(key, camera::CameraParam(v));
            }
        }, value);
    }

    return cam_config;
}

/**
 * @brief 将接收队列中的数据转移到缓冲区
 */
void drain_queue_to_buffer(serial::ReceiveQueue& queue,
                           std::deque<TimestampedSerialData>& buffer,
                           size_t max_buffer_size) {
    while (!queue.empty()) {
        TimestampedSerialData ts_data;
        ts_data.recv_time = SteadyClock::now();
        ts_data.data = queue.front();
        queue.pop();

        buffer.push_back(ts_data);
        while (buffer.size() > max_buffer_size) {
            buffer.pop_front();
        }
    }
}

/**
 * @brief 查找最接近目标时间的串口数据
 */
std::optional<serial::SerialReceiveData> find_closest_serial_data(
    const std::deque<TimestampedSerialData>& buffer,
    TimePoint target_time,
    int64_t max_diff_us = 50000) {
    if (buffer.empty()) return std::nullopt;

    auto best = buffer.begin();
    int64_t min_diff = INT64_MAX;

    for (auto it = buffer.begin(); it != buffer.end(); ++it) {
        int64_t diff = std::abs(
            std::chrono::duration_cast<std::chrono::microseconds>(
                it->recv_time - target_time
            ).count()
        );
        if (diff < min_diff) {
            min_diff = diff;
            best = it;
        }
    }

    if (min_diff <= max_diff_us) {
        return best->data;
    }
    return std::nullopt;
}

// ============================================================================
// Hardware Node Main Function
// ============================================================================

void start_hardware_node() {
    if (debug::get_session_path().empty()) {
        debug::init_session();
    }

    debug::print(debug::PrintMode::INFO, "HardwareNode", "Starting hardware node...");
    debug::print(debug::PrintMode::INFO, "HardwareNode", "Session: {}", debug::get_session_path());

    try {
        // Load config
        auto config = static_param::parse_file("hardware.toml");

        // Serial config
        std::string port_name = static_param::get_param<std::string>(config, "Serial", "port_name");
        int64_t baudrate = static_param::get_param<int64_t>(config, "Serial", "baudrate");
        int64_t delta_t_us = static_param::get_param<int64_t>(config, "TimeSync", "delta_t_us");

        // Fake serial config
        bool use_fake_serial = static_param::get_param<bool>(config, "Serial", "use_fake_serial_data");
        serial::SerialReceiveData fake_data;  // 预加载fake数据
        if (use_fake_serial) {
            fake_data.yaw = static_cast<float>(
                static_param::get_param<double>(config, "Serial.fake_data", "yaw_deg"));
            fake_data.pitch = static_cast<float>(
                static_param::get_param<double>(config, "Serial.fake_data", "pitch_deg"));
            fake_data.roll = static_cast<float>(
                static_param::get_param<double>(config, "Serial.fake_data", "roll_deg"));
            fake_data.robot_id = static_cast<uint8_t>(
                static_param::get_param<int64_t>(config, "Serial.fake_data", "robot_id"));
            fake_data.enemy_color = static_cast<uint8_t>(
                static_param::get_param<int64_t>(config, "Serial.fake_data", "enemy_color"));
            fake_data.bullet_speed = static_cast<float>(
                static_param::get_param<double>(config, "Serial.fake_data", "bullet_speed"));
            fake_data.aim_mode = static_cast<uint8_t>(
                static_param::get_param<int64_t>(config, "Serial.fake_data", "aim_mode"));
            fake_data.allow_fire = static_param::get_param<bool>(config, "Serial.fake_data", "allow_fire");
        }

        debug::print(debug::PrintMode::INFO, "HardwareNode", "Serial: {} @ {}", port_name, baudrate);
        debug::print(debug::PrintMode::INFO, "HardwareNode", "Delta_t: {} us", delta_t_us);
        debug::print(debug::PrintMode::INFO, "HardwareNode", "Use fake serial: {}", use_fake_serial);

        // 1. Start serial communication (only if not using fake)
        if (!use_fake_serial) {
            serial::start_serial_communication(port_name, static_cast<int>(baudrate));
            std::this_thread::sleep_for(100ms);  // Wait for serial threads to start
        } else {
            debug::print(debug::PrintMode::WARNING, "HardwareNode",
                "Using fake serial: color={}, bullet_speed={:.1f}",
                fake_data.enemy_color, fake_data.bullet_speed);
            // 创建空的接收队列
            umt::BasicObjManager<serial::ReceiveQueue>::find_or_create("receive_queue");
        }

        // 2. Load camera config and open camera
        camera::CameraConfig cam_config = load_camera_config(config);
        camera::HikCam cam(cam_config);
        cam.open();

        // 3. Setup UMT
        umt::Publisher<SyncFrame> pub("sync_frame");
        auto recv_queue = umt::BasicObjManager<serial::ReceiveQueue>::find_or_create("receive_queue");

        // 通知其他线程硬件节点已开始发布（初始为false，发布后设为true）
        auto hardware_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);

        debug::print(debug::PrintMode::LOG, "HardwareNode", "Hardware node started");

        // 串口数据缓冲区，用于时间同步匹配
        std::deque<TimestampedSerialData> serial_buffer;
        constexpr size_t max_buffer_size = 500;

        // FPS统计
        stats::FpsStats stats("HardwareNode", "synced");
        stats.set_extra_info([&serial_buffer]() {
            return fmt::format("buf: {}", serial_buffer.size());
        });

        // 4. Main loop
        while (true) {
            try {
                // Capture image
                cv::Mat& img = cam.capture();
                if (img.empty()) continue;

                TimePoint cam_time = SteadyClock::now();

                // Build sync frame
                SyncFrame frame;
                frame.image = img.clone();
                frame.frame_id = cam.frame_id;
                frame.timestamp = cam_time;

                // 根据是否使用fake serial决定数据来源
                bool synced = false;
                if (use_fake_serial) {
                    frame.serial_data = fake_data;
                    frame.serial_valid = true;
                    synced = true;
                } else {
                    drain_queue_to_buffer(recv_queue->get(), serial_buffer, max_buffer_size);

                    auto target = cam_time - std::chrono::microseconds(delta_t_us);
                    if (auto data = find_closest_serial_data(serial_buffer, target)) {
                        frame.serial_data = *data;
                        frame.serial_valid = true;
                        synced = true;
                    }
                }

                // Publish
                pub.push(frame);
                hardware_running->get() = true;

                // 更新统计
                stats.update(0, synced);

                // 更新 Dashboard 数据
                dashboard::set("hardware.fps", stats.last_fps);

            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::ERROR, "HardwareNode", "Loop error: {}", e.what());
                std::this_thread::sleep_for(100ms);
            }
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "HardwareNode", "Init failed: {}", e.what());
    }
}

void wait_hardware() {
    auto hardware_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    while (!hardware_running->get()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

} // namespace hardware
