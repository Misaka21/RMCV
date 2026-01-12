// C++ system headers
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

// Third-party library headers
#include <fmt/color.h>

#ifdef ENABLE_WEBVIEW
#include <Python.h>
#endif

// Project headers
#include "aimer/auto_aim/detector/detector_node.hpp"
#include "aimer/auto_aim/predictor/predictor_node.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "aimer/fire_control/fire_controller_node.hpp"
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/rmcv_bag/recorder_node.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "umt/umt.hpp"

#ifndef WEB_DIR
#define WEB_DIR "web"
#endif

void signal_handler(int sig) {
    fmt::print(fmt::fg(fmt::color::yellow), "\n[INFO] 收到信号 {}, 正在退出...\n", sig);
    auto app_running = umt::BasicObjManager<bool>::find("app_running");
    if (app_running) app_running->get() = false;
}

void print_usage(const char* prog_name) {
    fmt::print("用法: {} [选项]\n", prog_name);
    fmt::print("选项:\n");
    fmt::print("  --match, -m         比赛模式 (强制内录)\n");
    fmt::print("  --web               Web调试模式 (启动Flask服务器)\n");
    fmt::print("  --log-dir <path>    指定日志目录 (由 watchdog 传入)\n");
    fmt::print("  --help, -h          显示帮助信息\n");
}

int main(int argc, char* argv[]) {
    // ========== 解析命令行参数 ==========
    bool match_mode = false;
    bool web_mode = false;
    std::string log_dir;  // 外部指定的日志目录

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--match") == 0 || std::strcmp(argv[i], "-m") == 0) {
            match_mode = true;
        } else if (std::strcmp(argv[i], "--web") == 0) {
#ifdef ENABLE_WEBVIEW
            web_mode = true;
#else
            fmt::print(fmt::fg(fmt::color::red),
                "[ERROR] --web 需要 ENABLE_WEBVIEW，请安装 pybind11 后重新编译\n");
            return 1;
#endif
        } else if (std::strcmp(argv[i], "--log-dir") == 0 && i + 1 < argc) {
            log_dir = argv[++i];
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    // 设置全局比赛模式标志 (其他节点可以读取)
    auto match_mode_flag = umt::BasicObjManager<bool>::find_or_create("match_mode", match_mode);
    match_mode_flag->get() = match_mode;

    // 创建全局运行标志 (所有线程共享)
    auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    // 注册信号处理
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 初始化日志系统
    // watchdog 模式: 传入 --log-dir，直接使用
    // 直接运行模式: 自动创建带时间戳的目录
    std::string session_path;
    if (!log_dir.empty()) {
        session_path = debug::init_session(log_dir);
    } else {
        session_path = debug::init_session();
    }

    if (match_mode) {
        fmt::print(fmt::fg(fmt::color::red),
            "======================================================================\n"
            "                     MATCH MODE - 比赛模式                           \n"
            "                      强制内录已启用                                  \n"
            "======================================================================\n");
    }

    // 启动运行时参数热重载线程
    std::thread param_thread([]() {
        runtime_param::parameter_run("aimer.toml");
    });

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
    while (!hardware_ready->get() && app_running->get()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!app_running->get()) {
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

    // 启动火控节点线程
    std::thread fire_control_thread([]() {
        fire_control::start_fire_control("aimer.toml");
    });

    // 启动录制节点线程
    std::thread recorder_thread([]() {
        rmcv_bag::start_recorder_node();
    });

    // 启动看门狗节点线程
    std::string heartbeat_file = debug::get_session_path() + "/heartbeat";
    std::thread watchdog_thread([heartbeat_file]() {
        watchdog::start_watchdog_node(heartbeat_file, 5000, 1000);
    });

    debug::print(debug::PrintMode::INFO, "Main", "All threads started");
    fmt::print(fmt::fg(fmt::color::green), "[INFO] 系统启动完成，按 Ctrl+C 退出\n");

#ifdef ENABLE_WEBVIEW
    if (web_mode) {
        // Web调试模式：启动Flask服务器
        fmt::print(fmt::fg(fmt::color::cyan),
            "======================================================================\n"
            "                     WEB DEBUG MODE - 网页调试模式                    \n"
            "                     http://localhost:5000                            \n"
            "======================================================================\n");

        // 监控线程：当 app_running 变为 false 时强制退出
        // (因为 Py_Main 会阻塞，无法响应信号)
        std::thread exit_monitor([&app_running]() {
            while (app_running->get()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            // 等待其他线程有时间清理
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            fmt::print(fmt::fg(fmt::color::yellow), "[INFO] 强制退出 Web 模式\n");
            std::exit(0);
        });
        exit_monitor.detach();

        // 设置Python路径
        std::string app_path = std::string(WEB_DIR) + "/app.py";

        // 构建Python argv
        std::wstring w_prog = L"RMCV2026";
        std::wstring w_app_path(app_path.begin(), app_path.end());

        wchar_t* py_argv[] = {
            const_cast<wchar_t*>(w_prog.c_str()),
            const_cast<wchar_t*>(w_app_path.c_str()),
            nullptr
        };

        // 运行Python (阻塞，直到Flask服务器退出)
        int py_result = Py_Main(2, py_argv);

        // Python退出后，通知所有线程停止
        app_running->get() = false;
        fmt::print(fmt::fg(fmt::color::yellow), "[INFO] Flask服务器已退出 (code={})\n", py_result);
    } else
#endif
    {
        // 普通模式：主线程等待
        while (app_running->get()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // 等待线程结束
    fmt::print(fmt::fg(fmt::color::yellow), "[INFO] 等待线程结束...\n");
    if (param_thread.joinable()) {
        param_thread.join();
    }
    if (hardware_thread.joinable()) {
        hardware_thread.join();
    }
    if (detector_thread.joinable()) {
        detector_thread.join();
    }
    if (predictor_thread.joinable()) {
        predictor_thread.join();
    }
    if (fire_control_thread.joinable()) {
        fire_control_thread.join();
    }
    if (recorder_thread.joinable()) {
        recorder_thread.join();
    }
    if (watchdog_thread.joinable()) {
        watchdog_thread.join();
    }

    fmt::print(fmt::fg(fmt::color::green), "[INFO] 程序正常退出\n");
    return 0;
}
