// 检查相机和IMU时间戳的实际顺序
#include <chrono>
#include <thread>
#include <iostream>
#include <fmt/format.h>

#include "hardware/hardware_node.hpp"
#include "umt/umt.hpp"

using namespace std::chrono_literals;

int main() {
    fmt::print("启动硬件节点...\n");
    std::thread hw_thread(hardware::start_hardware_node);
    hw_thread.detach();

    auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    while (!hardware_ready->get()) {
        std::this_thread::sleep_for(100ms);
    }

    fmt::print("订阅同步帧...\n");
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");

    fmt::print("\n开始采样（10秒）...\n");
    fmt::print("观察 cam_time 和 imu_time 的大小关系\n\n");

    int positive_count = 0;  // cam_time > imu_time 的次数
    int negative_count = 0;  // cam_time < imu_time 的次数
    int total_count = 0;

    auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start < 10s) {
        try {
            auto frame = sub.pop_for(1000);

            if (!frame.serial_valid) continue;

            // 相机时间戳 (从 SyncFrame 直接获取)
            int64_t cam_time_us = frame.timestamp_us;

            // IMU接收时间戳
            int64_t imu_time_us = frame.serial_data.recv_time_us;

            int64_t diff_us = cam_time_us - imu_time_us;

            if (diff_us > 0) {
                positive_count++;
            } else {
                negative_count++;
            }
            total_count++;

            if (total_count % 10 == 0) {
                fmt::print("样本 {:3d}: cam - imu = {:+7.3f} ms\n",
                    total_count, diff_us / 1000.0);
            }

        } catch (const umt::MessageError_Timeout&) {
            continue;
        }
    }

    fmt::print("\n========== 统计结果 ==========\n");
    fmt::print("总样本数: {}\n", total_count);
    fmt::print("cam_time > imu_time: {} 次 ({:.1f}%)\n",
        positive_count, 100.0 * positive_count / total_count);
    fmt::print("cam_time < imu_time: {} 次 ({:.1f}%)\n",
        negative_count, 100.0 * negative_count / total_count);
    fmt::print("\n");

    if (positive_count > negative_count) {
        fmt::print("结论: 相机时间戳普遍晚于IMU\n");
        fmt::print("  → delta_t_us 应该为正值\n");
        fmt::print("  → hardware_node: target = cam_time - delta_t (查找过去的IMU)\n");
    } else {
        fmt::print("结论: 相机时间戳普遍早于IMU\n");
        fmt::print("  → delta_t_us 应该为负值\n");
        fmt::print("  → hardware_node: target = cam_time - delta_t (查找未来的IMU)\n");
    }

    return 0;
}
