//
// Created by RMCV on 2025/12/17.
// Hardware Node - 硬件节点管理器
// 负责启动串口、相机采集和数据同步打包
//

// C++ system headers
#include <chrono>
#include <cmath>
#include <deque>
#include <thread>

// Third-party library headers
#include <opencv2/core/mat.hpp>

// Project headers
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "hik_cam/hik_camera.hpp"
#include "serial/serial_thread.hpp"
#include "umt/umt.hpp"

namespace hardware {

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using TimePoint = std::chrono::steady_clock::time_point;

// ============================================================================
// Data Structures
// ============================================================================

struct ImuData {
    TimePoint timestamp;
    float yaw   = 0;
    float pitch = 0;
    float roll  = 0;
};

struct SyncFrame {
    cv::Mat image;
    int frame_id        = 0;
    TimePoint timestamp;
    ImuData imu;
    bool imu_valid      = false;
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
        double fake_yaw = static_param::get_param<double>(config, "Serial.data", "yaw_deg");

        debug::print(debug::PrintMode::INFO, "HardwareNode", "Serial: {} @ {}", port_name, baudrate);
        debug::print(debug::PrintMode::INFO, "HardwareNode", "Delta_t: {} us", delta_t_us);
        debug::print(debug::PrintMode::INFO, "HardwareNode", "Use fake serial: {}", use_fake_serial);

        // 1. Start serial communication (only if not using fake)
        if (!use_fake_serial) {
            serial::start_serial_communication(port_name, static_cast<int>(baudrate));
            std::this_thread::sleep_for(100ms);  // Wait for serial threads to start
        } else {
            debug::print(debug::PrintMode::WARNING, "HardwareNode", "Using fake serial data: yaw={}", fake_yaw);
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
        auto running = umt::BasicObjManager<bool>::find_or_create("hardware_running", true);

        // 通知主线程硬件初始化完成
        auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_ready", false);
        hardware_ready->get() = true;

        debug::print(debug::PrintMode::LOG, "HardwareNode", "Hardware node started");

        // IMU buffer for time matching
        std::deque<ImuData> imu_buffer;
        constexpr size_t max_buffer_size = 500;

        // FPS stats
        int fps_count = 0, sync_count = 0;
        auto fps_time = SteadyClock::now();

        // 4. Main loop
        while (running->get()) {
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

                // 根据是否使用fake serial决定IMU数据来源
                if (use_fake_serial) {
                    // 使用配置文件中的固定值
                    frame.imu.yaw = static_cast<float>(fake_yaw);
                    frame.imu.pitch = 0.0f;
                    frame.imu.timestamp = cam_time;
                    frame.imu_valid = true;
                    sync_count++;
                } else {
                    // Drain receive queue into IMU buffer
                    auto& queue = recv_queue->get();
                    while (!queue.empty()) {
                        auto data = queue.front();
                        queue.pop();

                        ImuData imu;
                        imu.timestamp = SteadyClock::now();
                        imu.yaw = data.yaw;
                        imu.pitch = data.pitch;

                        imu_buffer.push_back(imu);
                        while (imu_buffer.size() > max_buffer_size) {
                            imu_buffer.pop_front();
                        }
                    }

                    // Match closest IMU data
                    if (!imu_buffer.empty()) {
                        auto target = cam_time - std::chrono::microseconds(delta_t_us);

                        auto best = imu_buffer.begin();
                        int64_t min_diff = INT64_MAX;

                        for (auto it = imu_buffer.begin(); it != imu_buffer.end(); ++it) {
                            int64_t diff = std::abs(
                                std::chrono::duration_cast<std::chrono::microseconds>(
                                    it->timestamp - target
                                ).count()
                            );
                            if (diff < min_diff) {
                                min_diff = diff;
                                best = it;
                            }
                        }

                        // Accept if within 50ms
                        if (min_diff <= 50000) {
                            frame.imu = *best;
                            frame.imu_valid = true;
                            sync_count++;
                        }
                    }
                }

                // Publish
                pub.push(frame);

                // FPS stats
                fps_count++;
                auto now = SteadyClock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_time).count() >= 1000) {
                    debug::print(debug::PrintMode::DEBUG, "HardwareNode",
                                 "FPS: {}, synced: {}, imu_buf: {}", fps_count, sync_count, imu_buffer.size());
                    fps_count = 0;
                    sync_count = 0;
                    fps_time = now;
                }

            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::ERROR, "HardwareNode", "Loop error: {}", e.what());
                std::this_thread::sleep_for(100ms);
            }
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "HardwareNode", "Init failed: {}", e.what());
    }
}

} // namespace hardware
