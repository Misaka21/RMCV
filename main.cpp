/**
 * @file main.cpp
 * @brief RMCV 2026 主程序入口
 */

#include "main.hpp"

#include <csignal>
#include <cstring>
#include <thread>
#include <vector>

#ifdef ENABLE_WEBVIEW
#include <Python.h>
#endif

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
    fmt::print("  --match, -m            比赛模式 (强制内录)\n");
    fmt::print("  --minimal              最小模式 (无预测/录制/看门狗)\n");
    fmt::print("  --no-recorder          不启动录制\n");
    fmt::print("  --playback <path>      回放模式 (指定 bag 目录)\n");
    fmt::print("  --web                  Web调试模式 (启动Flask服务器)\n");
    fmt::print("  --log-dir <path>       指定日志目录 (由 watchdog 传入)\n");
    fmt::print("  --help, -h             显示帮助信息\n");
}

int main(int argc, char* argv[]) {
    using namespace std::chrono_literals;

    // ========== 解析命令行参数 ==========
    bool match_mode = false;
    bool web_mode = false;
    std::string log_dir;
    std::string playback_path;
    std::string run_mode = "default";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--match" || arg == "-m") {
            match_mode = true;
        } else if (arg == "--minimal") {
            run_mode = "--minimal";
        } else if (arg == "--no-recorder") {
            run_mode = "--no-recorder";
        } else if (arg == "--playback" && i + 1 < argc) {
            playback_path = argv[++i];
            run_mode = "--playback";
        } else if (arg == "--web") {
#ifdef ENABLE_WEBVIEW
            web_mode = true;
#else
            fmt::print(fmt::fg(fmt::color::red),
                "[ERROR] --web 需要 ENABLE_WEBVIEW，请安装 pybind11 后重新编译\n");
            return 1;
#endif
        } else if (arg == "--log-dir" && i + 1 < argc) {
            log_dir = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (MODE_THREADS.count(arg)) {
            run_mode = arg;
        }
    }

    // ========== 初始化全局标志 ==========
    auto match_mode_flag = umt::BasicObjManager<bool>::find_or_create("match_mode", match_mode);
    match_mode_flag->get() = match_mode;

    auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    // 存储回放路径 (供 playback 线程读取)
    if (!playback_path.empty()) {
        auto path_obj = umt::BasicObjManager<std::string>::find_or_create("playback_path", playback_path);
        path_obj->get() = playback_path;
    }

    // 注册信号处理
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ========== 初始化日志系统 ==========
    std::string session_path;
    if (!log_dir.empty()) {
        session_path = debug::init_session(log_dir);
    } else if (match_mode) {
        session_path = debug::init_session("", "match");
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

    fmt::print(fmt::fg(fmt::color::gold),
        "======================================================================\n"
        "                          RMCV 2026 启动                             \n"
        "======================================================================\n");

    // ========== 获取要启动的线程列表 ==========
    const auto& threads_to_run = MODE_THREADS.count(run_mode)
        ? MODE_THREADS.at(run_mode)
        : MODE_THREADS.at("default");

    debug::print(debug::PrintMode::INFO, "Main", "Starting threads: {}",
        fmt::join(threads_to_run, ", "));

    // ========== 启动线程 ==========
    std::vector<std::thread> threads;
    threads.reserve(threads_to_run.size());

    for (const auto& name : threads_to_run) {
        if (!THREAD_MAP.count(name)) {
            debug::print(debug::PrintMode::ERROR, "Main", "Unknown thread: {}", name);
            continue;
        }

        // 启动线程
        threads.emplace_back(THREAD_MAP.at(name));
        std::this_thread::sleep_for(10ms);

        // 如果有等待函数，执行等待
        if (WAIT_MAP.count(name)) {
            WAIT_MAP.at(name)();
            if (!app_running->get()) break;  // 被中断则提前退出
        }
    }

    if (!app_running->get()) {
        // 被中断，等待已启动的线程结束
        for (auto& t : threads) {
            if (t.joinable()) t.join();
        }
        return 0;
    }

    debug::print(debug::PrintMode::INFO, "Main", "All {} threads started", threads.size());
    fmt::print(fmt::fg(fmt::color::green), "[INFO] 系统启动完成，按 Ctrl+C 退出\n");

    // ========== 主循环 ==========
#ifdef ENABLE_WEBVIEW
    if (web_mode) {
        fmt::print(fmt::fg(fmt::color::cyan),
            "======================================================================\n"
            "                   WEB DEBUG MODE - 网页调试模式                     \n"
            "                     http://localhost:5000                           \n"
            "======================================================================\n");

        std::string app_path = std::string(WEB_DIR) + "/app.py";
        std::wstring w_prog = L"RMCV2026";
        std::wstring w_app_path(app_path.begin(), app_path.end());

        wchar_t* py_argv[] = {
            const_cast<wchar_t*>(w_prog.c_str()),
            const_cast<wchar_t*>(w_app_path.c_str()),
            nullptr
        };

        int py_result = Py_Main(2, py_argv);
        app_running->get() = false;
        fmt::print(fmt::fg(fmt::color::yellow), "[INFO] Flask服务器已退出 (code={})\n", py_result);
    } else
#endif
    {
        while (app_running->get()) {
            std::this_thread::sleep_for(100ms);
        }
    }

    // ========== 等待线程结束 ==========
    fmt::print(fmt::fg(fmt::color::yellow), "[INFO] 等待线程结束...\n");
    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }

    fmt::print(fmt::fg(fmt::color::green), "[INFO] 程序正常退出\n");
    return 0;
}
