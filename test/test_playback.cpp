/**
 * @file test_playback.cpp
 * @brief 测试回放节点
 *
 * 用法: ./test_playback <bag_path> [speed]
 */

#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>

#include <fmt/color.h>

#include "aimer/auto_aim/detector/detector_node.hpp"
#include "aimer/auto_aim/predictor/predictor_node.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/rmcv_bag/playback_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "umt/umt.hpp"

static std::atomic<bool> g_running{true};

void signal_handler(int sig) {
    fmt::print(fmt::fg(fmt::color::yellow), "\n[INFO] Signal {}, exiting...\n", sig);
    g_running = false;

    auto hw_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    auto det_running = umt::BasicObjManager<bool>::find_or_create("detector_running", false);
    auto pred_running = umt::BasicObjManager<bool>::find_or_create("predictor_running", false);
    hw_running->get() = false;
    det_running->get() = false;
    pred_running->get() = false;
    std::exit(0);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fmt::print("Usage: {} <bag_path> [speed]\n", argv[0]);
        fmt::print("  bag_path: directory containing raw.mkv and imu.csv\n");
        fmt::print("  speed: playback speed (default 1.0, 0 = no delay)\n");
        return 1;
    }

    std::string bag_path = argv[1];
    double speed = (argc > 2) ? std::stod(argv[2]) : 1.0;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    debug::init_session("test_playback");

    // 启动参数热重载
    std::thread param_thread([]() {
        runtime_param::parameter_run("aimer.toml");
    });
    param_thread.detach();
    runtime_param::wait_for_param("ok");

    // 初始化坐标变换
    tf::init("camera.yaml");

    fmt::print(fmt::fg(fmt::color::gold),
        "====================================================================\n"
        "                    RMCV 2026 Playback Mode\n"
        "====================================================================\n");

    fmt::print(fmt::fg(fmt::color::cyan),
        "Bag: {}\nSpeed: {}x\n", bag_path, speed);

    // 先初始化运行标志为 true
    auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_running", true);
    hardware_ready->get() = true;

    // 启动回放节点
    std::thread playback_thread([&]() {
        rmcv_bag::start_playback_node(bag_path, speed);
    });

    // 等待回放节点就绪
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 启动检测器
    std::thread detector_thread([]() {
        autoaim::start_detector_node();
    });

    // 启动预测器
    std::thread predictor_thread([]() {
        autoaim::predictor::start_predictor_node();
    });

    fmt::print(fmt::fg(fmt::color::green), "[INFO] Playback started, Ctrl+C to exit\n");

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (playback_thread.joinable()) playback_thread.join();
    if (detector_thread.joinable()) detector_thread.join();
    if (predictor_thread.joinable()) predictor_thread.join();

    fmt::print(fmt::fg(fmt::color::green), "[INFO] Done\n");
    return 0;
}
