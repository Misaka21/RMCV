//
// Hardware Node - 硬件节点管理器
// 负责启动串口、相机采集和数据同步打包
//

#ifndef HARDWARE_NODE_HPP
#define HARDWARE_NODE_HPP

// C++ 标准库
#include <chrono>
#include <cmath>

// 第三方库
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

// 项目头文件
#include "serial/serial_thread.hpp"

namespace hardware {

using TimePoint = std::chrono::steady_clock::time_point;

/**
 * @brief IMU数据，支持四元数转换
 */
struct ImuData {
    float yaw   = 0;  // 偏航角 (°)
    float pitch = 0;  // 俯仰角 (°)
    float roll  = 0;  // 横滚角 (°)

    ImuData() = default;
    ImuData(float y, float p, float r) : yaw(y), pitch(p), roll(r) {}

    // 欧拉角转四元数 (ZYX顺序)
    Eigen::Quaterniond to_quaternion() const {
        constexpr double deg2rad = M_PI / 180.0;
        double cy = std::cos(yaw * deg2rad * 0.5);
        double sy = std::sin(yaw * deg2rad * 0.5);
        double cp = std::cos(pitch * deg2rad * 0.5);
        double sp = std::sin(pitch * deg2rad * 0.5);
        double cr = std::cos(roll * deg2rad * 0.5);
        double sr = std::sin(roll * deg2rad * 0.5);

        Eigen::Quaterniond q;
        q.w() = cy * cp * cr + sy * sp * sr;
        q.x() = cy * cp * sr - sy * sp * cr;
        q.y() = cy * sp * cr + sy * cp * sr;
        q.z() = sy * cp * cr - cy * sp * sr;
        return q;
    }

    // 获取旋转矩阵
    Eigen::Matrix3d to_rotation_matrix() const {
        return to_quaternion().toRotationMatrix();
    }
};

/**
 * @brief 同步帧 - 相机图像 + 串口数据打包
 */
struct SyncFrame {
    cv::Mat image;
    int frame_id = 0;
    TimePoint timestamp;  // 图像采集时间

    serial::SerialReceiveData serial_data;
    bool serial_valid = false;

    // 便捷获取IMU数据
    ImuData imu() const {
        return ImuData(serial_data.yaw, serial_data.pitch, serial_data.roll);
    }
};

/**
 * @brief 启动硬件节点
 * 1. 启动串口收发线程
 * 2. 打开相机
 * 3. 主循环：采集图像 -> 匹配串口数据 -> 发布SyncFrame
 */
void start_hardware_node();

/**
 * @brief 等待硬件节点初始化完成
 */
void wait_hardware();

} // namespace hardware

#endif // HARDWARE_NODE_HPP
