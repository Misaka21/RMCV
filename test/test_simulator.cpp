//
// test_simulator.cpp - 模拟器接入测试
// 用于benchmark测试，替代真实硬件
//
// 使用方法:
//   1. 启动模拟器: cd at_vision_simulator && cargo run --release
//   2. 运行测试: ./test_simulator
//

#include <csignal>
#include <iostream>
#include <thread>

#include <fmt/color.h>
#include <opencv2/highgui.hpp>

#include "simulator/simulator_node.hpp"
#include "aimer/auto_aim/detector/detector_node.hpp"
#include "aimer/auto_aim/predictor/predictor_node.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "umt/umt.hpp"

static std::atomic<bool> g_running{true};

void signal_handler(int sig) {
    fmt::print(fmt::fg(fmt::color::yellow), "\n[INFO] Signal {}, exiting...\n", sig);
    g_running = false;
    std::exit(0);
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    debug::init_session();

    fmt::print(fmt::fg(fmt::color::gold),
        "====================================================================\n"
        "                    RMCV Simulator Benchmark\n"
        "====================================================================\n");

    // 启动参数热重载
    std::thread param_thread([]() {
        runtime_param::parameter_run("aimer.toml");
    });
    param_thread.detach();
    runtime_param::wait_for_param("ok");

    // 初始化坐标变换
    tf::init("camera.yaml");

    auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);

    // 启动simulator节点 (替代hardware节点)
    fmt::print(fmt::fg(fmt::color::cyan), "[INFO] Starting simulator node...\n");
    fmt::print(fmt::fg(fmt::color::cyan), "[INFO] Make sure at_vision_simulator is running!\n");

    std::thread simulator_thread([]() {
        simulator::start_simulator_node();
    });

    // 等待simulator就绪
    fmt::print("[INFO] Waiting for simulator data...\n");
    while (!hardware_ready->get() && g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!g_running) {
        if (simulator_thread.joinable()) simulator_thread.join();
        return 0;
    }

    fmt::print(fmt::fg(fmt::color::green), "[INFO] Simulator ready, starting detector...\n");

    // 启动检测器
    std::thread detector_thread([]() {
        autoaim::start_detector_node();
    });

    // 启动预测器
    fmt::print(fmt::fg(fmt::color::green), "[INFO] Starting predictor...\n");
    std::thread predictor_thread([]() {
        autoaim::predictor::start_predictor_node();
    });

    fmt::print(fmt::fg(fmt::color::green), "[INFO] Benchmark running. Press Ctrl+C or 'q' to stop.\n");

    // 主线程处理 OpenCV GUI 事件 (必须在主线程)
    while (g_running) {
        int key = cv::waitKey(1);  // 30ms，约33fps
        if (key == 'q' || key == 27) {  // q 或 ESC
            g_running = false;
            break;
        }
    }

    if (simulator_thread.joinable()) simulator_thread.join();
    if (detector_thread.joinable()) detector_thread.join();
    if (predictor_thread.joinable()) predictor_thread.join();

    fmt::print(fmt::fg(fmt::color::green), "[INFO] Done.\n");
    return 0;
}
