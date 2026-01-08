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
#include <Eigen/Geometry>  // for Quaterniond SLERP

// Project headers
#include "hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "hik_cam/hik_camera.hpp"
#include "serial/serial_thread.hpp"
#include "umt/umt.hpp"

namespace hardware {

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

// 带时间戳的串口数据，用于时间同步匹配
struct TimestampedSerialData {
    int64_t recv_time_us;  // 接收时间 (微秒)
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
 * 使用串口线程记录的精确时间戳
 */
void drain_queue_to_buffer(serial::ReceiveQueue& queue,
                           std::deque<TimestampedSerialData>& buffer,
                           size_t max_buffer_size) {
    while (!queue.empty()) {
        TimestampedSerialData ts_data;
        ts_data.data = queue.front();
        ts_data.recv_time_us = ts_data.data.recv_time_us;  // 使用串口线程记录的时间戳
        queue.pop();

        buffer.push_back(ts_data);
        while (buffer.size() > max_buffer_size) {
            buffer.pop_front();
        }
    }
}

/**
 * @brief 欧拉角转四元数 (ZYX顺序: yaw-pitch-roll)
 */
inline Eigen::Quaterniond euler_to_quat(float yaw_deg, float pitch_deg, float roll_deg) {
    double yaw = yaw_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double roll = roll_deg * M_PI / 180.0;
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

/**
 * @brief 四元数转欧拉角 (ZYX顺序)
 */
inline void quat_to_euler(const Eigen::Quaterniond& q, float& yaw_deg, float& pitch_deg, float& roll_deg) {
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX
    yaw_deg = static_cast<float>(euler[0] * 180.0 / M_PI);
    pitch_deg = static_cast<float>(euler[1] * 180.0 / M_PI);
    roll_deg = static_cast<float>(euler[2] * 180.0 / M_PI);
}

/**
 * @brief 插值获取目标时间的串口数据
 * 对 yaw/pitch/roll 做 SLERP 球面插值，其他字段取最近的
 * 支持边界外推（当目标时间超出缓冲区范围时）
 */
std::optional<serial::SerialReceiveData> interpolate_serial_data(
    const std::deque<TimestampedSerialData>& buffer,
    int64_t target_time_us,
    int64_t max_diff_us = 50000) {

    if (buffer.size() < 2) return std::nullopt;

    // 找到第一个 >= target_time 的位置
    auto it_after = std::lower_bound(
        buffer.begin(), buffer.end(), target_time_us,
        [](const TimestampedSerialData& d, int64_t t) {
            return d.recv_time_us < t;
        }
    );

    const TimestampedSerialData* before_ptr = nullptr;
    const TimestampedSerialData* after_ptr = nullptr;

    if (it_after == buffer.begin()) {
        // target 在所有数据之前，用最早的两个外推
        before_ptr = &buffer[0];
        after_ptr = &buffer[1];
    } else if (it_after == buffer.end()) {
        // target 在所有数据之后，用最新的两个外推
        before_ptr = &buffer[buffer.size() - 2];
        after_ptr = &buffer[buffer.size() - 1];
    } else {
        // 正常情况：target 在两个数据之间
        before_ptr = &(*std::prev(it_after));
        after_ptr = &(*it_after);
    }

    const auto& before = *before_ptr;
    const auto& after = *after_ptr;

    // 检查时间差是否在允许范围内
    int64_t min_diff = std::min(
        std::abs(before.recv_time_us - target_time_us),
        std::abs(after.recv_time_us - target_time_us)
    );
    if (min_diff > max_diff_us) {
        return std::nullopt;
    }

    // 计算插值因子 t（可能 < 0 或 > 1 表示外推）
    double dt = static_cast<double>(after.recv_time_us - before.recv_time_us);
    if (dt <= 0) {
        return before.data;
    }
    double t = static_cast<double>(target_time_us - before.recv_time_us) / dt;

    // 插值结果
    serial::SerialReceiveData result;

    // SLERP 姿态插值
    Eigen::Quaterniond q_before = euler_to_quat(before.data.yaw, before.data.pitch, before.data.roll);
    Eigen::Quaterniond q_after = euler_to_quat(after.data.yaw, after.data.pitch, after.data.roll);
    Eigen::Quaterniond q_interp = q_before.slerp(t, q_after);
    quat_to_euler(q_interp, result.yaw, result.pitch, result.roll);

    // 其他字段取最近的
    const auto& nearest = (t < 0.5) ? before.data : after.data;
    result.robot_id = nearest.robot_id;
    result.enemy_color = nearest.enemy_color;
    result.bullet_speed = nearest.bullet_speed;
    result.aim_mode = nearest.aim_mode;
    result.allow_fire = nearest.allow_fire;
    result.aiming_lock = nearest.aiming_lock;

    // 时间戳设为目标时间
    result.recv_time_us = target_time_us;

    return result;
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

        // IMU pitch/roll取反配置
        bool imu_pitch_negate = static_param::get_param<bool>(config, "Serial", "imu_pitch_negate");
        bool imu_roll_negate = static_param::get_param<bool>(config, "Serial", "imu_roll_negate");

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
            fake_data.aiming_lock = static_param::get_param<bool>(config, "Serial.fake_data", "aiming_lock");
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
        // 全局运行标志 (初始 true，退出时设为 false)
        auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

        debug::print(debug::PrintMode::INFO, "HardwareNode", "Hardware node started");

        // 串口数据缓冲区，用于时间同步匹配
        std::deque<TimestampedSerialData> serial_buffer;
        constexpr size_t max_buffer_size = 200;

        // FPS统计
        stats::FpsStats stats("HardwareNode", "synced");
        stats.set_extra_info([&serial_buffer]() {
            return fmt::format("buf: {}", serial_buffer.size());
        });

        // 4. Main loop
        int consecutive_errors = 0;
        const int MAX_CONSECUTIVE_ERRORS = 3;  // 连续失败3次后退出

        while (app_running->get()) {
            watchdog::heartbeat("hardware");
            try {
                // Capture image
                cv::Mat& img = cam.capture();
                if (img.empty()) continue;

                consecutive_errors = 0;  // 成功后重置计数

                int64_t cam_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    SteadyClock::now().time_since_epoch()
                ).count();

                // Build sync frame
                SyncFrame frame;
                frame.image = img.clone();
                frame.frame_id = cam.frame_id;
                frame.timestamp_us = cam_time_us;

                // 根据是否使用fake serial决定数据来源
                bool synced = false;
                if (use_fake_serial) {
                    frame.serial_data = fake_data;
                    frame.serial_valid = true;
                    synced = true;
                } else {
                    drain_queue_to_buffer(recv_queue->get(), serial_buffer, max_buffer_size);

                    int64_t target_time_us = cam_time_us - delta_t_us;
                    if (auto data = interpolate_serial_data(serial_buffer, target_time_us)) {
                        frame.serial_data = *data;
                        frame.serial_valid = true;
                        synced = true;
                    }
                }

                // 应用IMU pitch/roll取反
                if (frame.serial_valid) {
                    if (imu_pitch_negate) {
                        frame.serial_data.pitch = -frame.serial_data.pitch;
                    }
                    if (imu_roll_negate) {
                        frame.serial_data.roll = -frame.serial_data.roll;
                    }
                }

                // Publish
                pub.push(frame);

                // 只有串口同步后才通知其他线程硬件就绪
                if (frame.serial_valid && !hardware_running->get()) {
                    hardware_running->get() = true;
                    debug::print(debug::PrintMode::INFO, "HardwareNode", "Serial synced, hardware ready");
                }

                // 更新统计
                stats.update(0, synced);

            } catch (const std::exception& e) {
                consecutive_errors++;
                debug::print(debug::PrintMode::ERROR, "HardwareNode",
                    "Loop error ({}/{}): {}", consecutive_errors, MAX_CONSECUTIVE_ERRORS, e.what());

                if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                    debug::print(debug::PrintMode::FATAL, "HardwareNode",
                        "Too many consecutive errors, camera disconnected?");
                    std::exit(1);
                }
                std::this_thread::sleep_for(100ms);
            }
        }

        debug::print(debug::PrintMode::INFO, "HardwareNode", "Hardware node stopped");

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::FATAL, "HardwareNode", "Init failed: {}", e.what());
        std::exit(1);
    }
}

} // namespace hardware
