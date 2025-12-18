//
// RMCV 2026 - 主入口
// 比赛模式: ./RMCV2026
// 调试模式: ./RMCV2026 --py [script.py]
//

#include <Python.h>
#include <thread>
#include <string>

#include <fmt/color.h>

#include "hardware/hardware_node.hpp"
#include "aimer/auto_aim/detector/detector_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"

int main(int argc, char* argv[]) {
    debug::init_session();

    // 检查是否 Python 模式
    bool python_mode = false;
    int py_argc = 0;
    char** py_argv = nullptr;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--py") {
            python_mode = true;
            py_argc = argc - i;
            py_argv = argv + i;
            break;
        }
    }

    // 参数热加载
    fmt::print(fmt::fg(fmt::color::gold), "==================== Loading Parameters ====================\n");
    std::thread([]() { runtime_param::parameter_run("detector.toml"); }).detach();

    // 硬件节点 (相机 + 串口)
    fmt::print(fmt::fg(fmt::color::gold), "===================== Loading Hardware =====================\n");
    std::thread([]() { hardware::start_hardware_node(); }).detach();
    hardware::wait_hardware();

    // 检测器
    fmt::print(fmt::fg(fmt::color::gold), "===================== Loading Detector =====================\n");
    std::thread([]() { autoaim::start_detector_node(); }).detach();

    fmt::print(fmt::fg(fmt::color::green), "====================== System Ready ========================\n");

    if (python_mode) {
        // Python 模式
        fmt::print(fmt::fg(fmt::color::cyan), "[Python Mode]\n");
        wchar_t* w_argv[py_argc];
        for (int i = 0; i < py_argc; i++) {
            size_t len = std::mbstowcs(nullptr, py_argv[i], 0) + 1;
            w_argv[i] = new wchar_t[len];
            std::mbstowcs(w_argv[i], py_argv[i], len);
        }
        return Py_Main(py_argc, w_argv);
    } else {
        // 比赛模式
        fmt::print(fmt::fg(fmt::color::green), "[Race Mode] Press Ctrl+C to exit\n");
        while (true) {
            std::this_thread::sleep_for(std::chrono::hours(24));
        }
    }
}
