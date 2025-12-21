//
// IMU 方向测试
// 实时显示 yaw/pitch/roll，验证陀螺仪方向是否正确
//

#include <chrono>
#include <thread>
#include <fmt/format.h>
#include <fmt/color.h>

#include "hardware/hardware_node.hpp"
#include "umt/umt.hpp"

using namespace std::chrono_literals;

int main() {
    fmt::print(fmt::fg(fmt::color::gold),
        "====================================================================\n"
        "                     IMU 方向测试\n"
        "====================================================================\n"
        "预期行为:\n"
        "  左转 (yaw)   → yaw 增大 ↑\n"
        "  右转 (yaw)   → yaw 减小 ↓\n"
        "  抬头 (pitch) → pitch 增大 ↑\n"
        "  低头 (pitch) → pitch 减小 ↓\n"
        "  右倾 (roll)  → roll 增大 ↑\n"
        "  左倾 (roll)  → roll 减小 ↓\n"
        "====================================================================\n\n"
    );

    // 启动硬件节点线程
    std::thread hw_thread([]() {
        hardware::start_hardware_node();
    });
    hw_thread.detach();

    // 等待硬件节点启动
    auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    while (!hardware_ready->get()) {
        std::this_thread::sleep_for(500ms);
    }

    // 订阅同步帧
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");

    float last_yaw = 0, last_pitch = 0, last_roll = 0;
    bool first = true;

    while (true) {
        try {
            auto frame = sub.pop_for(1000);

            if (!frame.serial_valid) {
                fmt::print(fmt::fg(fmt::color::yellow), "\r[等待串口数据...]");
                std::fflush(stdout);
                continue;
            }

            float yaw = frame.serial_data.yaw;
            float pitch = frame.serial_data.pitch;
            float roll = frame.serial_data.roll;

            // 计算变化方向
            auto arrow = [](float delta) -> const char* {
                if (delta > 0.5f) return "↑";
                if (delta < -0.5f) return "↓";
                return " ";
            };

            auto color = [](float delta) {
                if (delta > 0.5f) return fmt::fg(fmt::color::green);
                if (delta < -0.5f) return fmt::fg(fmt::color::red);
                return fmt::fg(fmt::color::white);
            };

            if (!first) {
                float dy = yaw - last_yaw;
                float dp = pitch - last_pitch;
                float dr = roll - last_roll;

                // 处理 yaw 跨越 ±180° 的情况
                if (dy > 180) dy -= 360;
                if (dy < -180) dy += 360;

                fmt::print("\r");
                fmt::print("yaw: ");
                fmt::print(color(dy), "{:+7.2f} {} ", yaw, arrow(dy));
                fmt::print("  pitch: ");
                fmt::print(color(dp), "{:+7.2f} {} ", pitch, arrow(dp));
                fmt::print("  roll: ");
                fmt::print(color(dr), "{:+7.2f} {} ", roll, arrow(dr));
                fmt::print("    ");
                std::fflush(stdout);
            }

            last_yaw = yaw;
            last_pitch = pitch;
            last_roll = roll;
            first = false;

        } catch (const umt::MessageError_Timeout&) {
            fmt::print(fmt::fg(fmt::color::yellow), "\r[超时等待帧...]");
            std::fflush(stdout);
        }
    }

    return 0;
}
