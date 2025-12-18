// C++ system headers
#include <csignal>
#include <iostream>
#include <thread>

// Third-party library headers
#include <fmt/color.h>

// Project headers
#include "aimer/auto_aim/detector/detector_node.hpp"
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "umt/umt.hpp"

// 全局运行标志
static std::atomic<bool> g_running{true};

void signal_handler(int sig) {
    fmt::print(fmt::fg(fmt::color::yellow), "\n[INFO] 收到信号 {}, 正在退出...\n", sig);
    g_running = false;

    // 通知所有线程停止
    auto hw_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    auto det_running = umt::BasicObjManager<bool>::find_or_create("detector_running", false);
    hw_running->get() = false;
    det_running->get() = false;
}

int main() {
    // 注册信号处理
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 初始化日志系统
    debug::init_session();

    fmt::print(fmt::fg(fmt::color::gold),
               "====================================================================\n"
               "                        RMCV 2026 启动\n"
               "====================================================================\n");

    debug::print(debug::PrintMode::INFO, "Main", "Starting RMCV 2026...");

    // 创建同步标志
    auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);

    // 启动硬件节点线程 (相机 + 串口/fake serial)
    std::thread hardware_thread([]() {
        hardware::start_hardware_node();
    });

    // 等待硬件节点完成初始化
    debug::print(debug::PrintMode::INFO, "Main", "Waiting for hardware node...");
    while (!hardware_ready->get() && g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!g_running) {
        if (hardware_thread.joinable()) hardware_thread.join();
        return 0;
    }

    debug::print(debug::PrintMode::INFO, "Main", "Hardware node ready, starting detector...");

    // 启动检测器节点线程
    std::thread detector_thread([]() {
        autoaim::start_detector_node(autoaim::detector::EnemyColor::RED);
    });

    debug::print(debug::PrintMode::INFO, "Main", "All threads started");
    fmt::print(fmt::fg(fmt::color::green), "[INFO] 系统启动完成，按 Ctrl+C 退出\n");

    // 主线程等待
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 等待线程结束
    fmt::print(fmt::fg(fmt::color::yellow), "[INFO] 等待线程结束...\n");
    if (hardware_thread.joinable()) {
        hardware_thread.join();
    }
    if (detector_thread.joinable()) {
        detector_thread.join();
    }

    fmt::print(fmt::fg(fmt::color::green), "[INFO] 程序正常退出\n");
    return 0;
}