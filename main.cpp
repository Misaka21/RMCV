// C++ system headers
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

// Third-party library headers
#include <fmt/color.h>

// Project headers
#include "aimer/auto_aim/detector/detector_node.hpp"
#include "aimer/auto_aim/predictor/predictor_node.hpp"
#include "aimer/auto_aim/fire_control/fire_control_node.hpp"
#include "aimer/auto_buff/detector/detector_node.hpp"
#include "aimer/auto_buff/predictor/predictor_node.hpp"
#include "aimer/auto_buff/fire_control/fire_control_node.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/rerun/rmcv_rerun.hpp"
#include "plugin/rmcv_bag/recorder_node.hpp"
#include "plugin/visualizer/visualizer_node.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "umt/umt.hpp"

// 全局原子标志，信号处理函数中安全使用（不走锁）
static std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int sig) {
    if (g_shutdown_requested.exchange(true)) {
        // 第二次 Ctrl+C: 强制退出
        fmt::print(fmt::fg(fmt::color::red), "\n[WARN] 再次收到信号 {}, 强制退出\n", sig);
        std::_Exit(1);
    }
    fmt::print(fmt::fg(fmt::color::yellow), "\n[INFO] 收到信号 {}, 正在退出...\n", sig);
}

void print_usage(const char* prog_name) {
    fmt::print("用法: {} [选项]\n", prog_name);
    fmt::print("选项:\n");
    fmt::print("  --match, -m         比赛模式 (强制内录)\n");
    fmt::print("  --log-dir <path>    指定日志目录 (由 watchdog 传入)\n");
    fmt::print("  --help, -h          显示帮助信息\n");
}

int main(int argc, char* argv[]) {
    // ========== 解析命令行参数 ==========
    bool match_mode = false;
    std::string log_dir;  // 外部指定的日志目录

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--match") == 0 || std::strcmp(argv[i], "-m") == 0) {
            match_mode = true;
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
    // 直接运行模式: 自动创建带时间戳的目录，比赛模式附加 match 后缀
    std::string session_path;
    if (!log_dir.empty()) {
        session_path = debug::init_session(log_dir);
    } else if (match_mode) {
        session_path = debug::init_session("match");
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
    std::thread buff_param_thread([]() {
        runtime_param::parameter_run("buff.toml");
    });

    // 等待参数加载完成 (任一文件完成即可)
    runtime_param::wait_for_param("ok");

    // 初始化坐标变换系统
    aimer::tf::init("camera.yaml");

    // 初始化 Rerun 可视化
    rr::init();

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
        autoaim::fire_control::start_fire_control_node("aimer.toml");
    });

    // 启动能量机关检测器节点线程
    std::thread buff_detector_thread([]() {
        autobuff::detector::background_buff_detector_run("buff_detector.toml");
    });

    // 启动能量机关预测器节点线程
    std::thread buff_predictor_thread([]() {
        autobuff::predictor::start_predictor_node();
    });

    // 启动能量机关火控节点线程
    std::thread buff_fire_control_thread([]() {
        autobuff::fire_control::start_fire_control_node("aimer.toml");
    });

    // 启动录制节点线程
    std::thread recorder_thread([]() {
        rmcv_bag::start_recorder_node();
    });

    // 启动可视化节点线程
    std::thread visualizer_thread([]() {
        visualizer::start_visualizer_node();
    });

    // 启动看门狗节点线程
    std::string heartbeat_file = debug::get_session_path() + "/heartbeat";
    std::thread watchdog_thread([heartbeat_file]() {
        watchdog::start_watchdog_node(heartbeat_file, 5000, 1000);
    });

    debug::print(debug::PrintMode::INFO, "Main", "All threads started");
    fmt::print(fmt::fg(fmt::color::green), "[INFO] 系统启动完成，按 Ctrl+C 退出\n");

    // 主线程等待
    while (app_running->get()) {
        if (g_shutdown_requested.load()) {
            app_running->get() = false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 等待线程结束 (带超时兜底)
    fmt::print(fmt::fg(fmt::color::yellow), "[INFO] 等待线程结束...\n");

    // 超时看门狗：如果 join 超时则强制退出
    constexpr int SHUTDOWN_TIMEOUT_S = 5;
    std::thread shutdown_watchdog([timeout = SHUTDOWN_TIMEOUT_S]() {
        std::this_thread::sleep_for(std::chrono::seconds(timeout));
        fmt::print(fmt::fg(fmt::color::red),
            "[WARN] 退出超时 ({}s)，强制退出\n", timeout);
        std::_Exit(0);
    });
    shutdown_watchdog.detach();

    if (param_thread.joinable()) {
        param_thread.join();
    }
    if (buff_param_thread.joinable()) {
        buff_param_thread.join();
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
    if (buff_detector_thread.joinable()) {
        buff_detector_thread.join();
    }
    if (buff_predictor_thread.joinable()) {
        buff_predictor_thread.join();
    }
    if (buff_fire_control_thread.joinable()) {
        buff_fire_control_thread.join();
    }
    if (recorder_thread.joinable()) {
        recorder_thread.join();
    }
    if (visualizer_thread.joinable()) {
        visualizer_thread.join();
    }
    if (watchdog_thread.joinable()) {
        watchdog_thread.join();
    }

    rr::shutdown();
    fmt::print(fmt::fg(fmt::color::green), "[INFO] 程序正常退出\n");
    return 0;
}
