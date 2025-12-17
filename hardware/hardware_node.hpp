//
// Created by RMCV on 2025/12/17.
// Hardware Node - 硬件节点管理器
// 负责启动串口、相机采集和数据同步打包
//

#ifndef HARDWARE_NODE_HPP
#define HARDWARE_NODE_HPP

// C++ system headers
#include <chrono>

// Third-party library headers
#include <opencv2/core/mat.hpp>

namespace hardware {

using TimePoint = std::chrono::steady_clock::time_point;

/**
 * @brief IMU data with timestamp
 */
struct ImuData {
    TimePoint timestamp;
    float yaw   = 0;
    float pitch = 0;
    float roll  = 0;
};

/**
 * @brief Synchronized frame - camera image + matched IMU data
 */
struct SyncFrame {
    cv::Mat image;
    int frame_id        = 0;
    TimePoint timestamp;
    ImuData imu;
    bool imu_valid      = false;
};

/**
 * @brief Start hardware node
 *
 * This function:
 * 1. Starts serial communication (send + receive threads)
 * 2. Opens camera
 * 3. Runs main loop: capture image -> match IMU -> publish SyncFrame
 *
 * Publishes SyncFrame to "sync_frame" channel via UMT
 * Config from hardware.toml: Serial.port_name, Serial.baudrate, TimeSync.delta_t_us
 */
void start_hardware_node();

} // namespace hardware

#endif // HARDWARE_NODE_HPP
