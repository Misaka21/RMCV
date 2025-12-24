//
// Hardware Node - 硬件节点管理器
// 负责启动串口、相机采集和数据同步打包
//

#ifndef HARDWARE_NODE_HPP
#define HARDWARE_NODE_HPP

#include <cstdint>

#include <opencv2/core/mat.hpp>

#include "serial/serial_thread.hpp"

namespace hardware {

/**
 * @brief 同步帧 - Hardware层输出
 *
 * 相机图像 + 时间戳匹配的串口数据
 * 这是最底层的数据结构，不依赖上层模块
 */
struct SyncFrame {
    cv::Mat image;
    int frame_id = 0;
    int64_t timestamp_us = 0;  // 时间戳 (微秒, steady_clock)

    serial::SerialReceiveData serial_data;
    bool serial_valid = false;
};

/**
 * @brief 启动硬件节点
 *
 * 发布消息: Message<hardware::SyncFrame> "sync_frame"
 */
void start_hardware_node();

}  // namespace hardware

#endif  // HARDWARE_NODE_HPP
