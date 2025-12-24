// C++ system headers
#include <csignal>
#include <iostream>
#include <thread>

// Third-party library headers
#include <fmt/color.h>

// Project headers
#include "aimer/auto_aim/detector/detector_node.hpp"
#include "aimer/auto_aim/predictor/predictor_node.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/rmcv_bag/recorder_node.hpp"
#include "umt/umt.hpp"

// 全局运行标志
static std::atomic<bool> g_running{true};

void signal_handler(int sig) {
    fmt::print(fmt::fg(fmt::color::yellow), "\n[INFO] 收到信号 {}, 正在退出...\n", sig);
    g_running = false;

    // 通知所有线程停止
    auto hw_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    auto det_running = umt::BasicObjManager<bool>::find_or_create("detector_running", false);
    auto pred_running = umt::BasicObjManager<bool>::find_or_create("predictor_running", false);
    auto rec_running = umt::BasicObjManager<bool>::find_or_create("recorder_running", false);
    hw_running->get() = false;
    det_running->get() = false;
    pred_running->get() = false;
    rec_running->get() = false;
    std::exit(1);
}

int main() {
    // 注册信号处理
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 初始化日志系统
    debug::init_session();

    // 启动运行时参数热重载线程 (内部有无限循环，必须在单独线程运行)
    std::thread param_thread([]() {
        runtime_param::parameter_run("aimer.toml");
    });
    param_thread.detach();  // 分离线程，程序退出时自动结束

    // 启动 recorder 参数热更新线程
    std::thread recorder_param_thread([]() {
        runtime_param::parameter_run("recorder.toml");
    });
    recorder_param_thread.detach();

    // 等待参数加载完成
    runtime_param::wait_for_param("ok");

    // 初始化坐标变换系统
    tf::init("camera.yaml");

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

    // 启动检测器节点线程 (颜色从串口获取，不使用默认值)
    std::thread detector_thread([]() {
        autoaim::start_detector_node();
    });

    // 启动预测器节点线程
    std::thread predictor_thread([]() {
        autoaim::predictor::start_predictor_node();
    });

    // 启动录制节点线程
    std::thread recorder_thread([]() {
        rmcv_bag::start_recorder_node();
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
    if (predictor_thread.joinable()) {
        predictor_thread.join();
    }
    if (recorder_thread.joinable()) {
        recorder_thread.join();
    }

    fmt::print(fmt::fg(fmt::color::green), "[INFO] 程序正常退出\n");
    return 0;
}
