/**
 * @file main.hpp
 * @brief 主程序线程注册表
 */

#ifndef RMCV_MAIN_HPP
#define RMCV_MAIN_HPP

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <fmt/color.h>

#include "aimer/auto_aim/detector/detector_node.hpp"
#include "aimer/auto_aim/predictor/predictor_node.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/rmcv_bag/playback_node.hpp"
#include "plugin/rmcv_bag/recorder_node.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "umt/umt.hpp"

// 线程启动函数映射表
inline const std::unordered_map<std::string, std::function<void()>> THREAD_MAP = {
    {"param", [] {
        fmt::print(fmt::fg(fmt::color::gold),
            "==================== Loading Parameters ====================\n");
        runtime_param::parameter_run("aimer.toml");
    }},
    {"hardware", [] {
        fmt::print(fmt::fg(fmt::color::gold),
            "==================== Loading Hardware ====================\n");
        hardware::start_hardware_node();
    }},
    {"detector", [] {
        fmt::print(fmt::fg(fmt::color::gold),
            "==================== Loading Detector ====================\n");
        autoaim::start_detector_node();
    }},
    {"predictor", [] {
        fmt::print(fmt::fg(fmt::color::gold),
            "==================== Loading Predictor ====================\n");
        autoaim::predictor::start_predictor_node();
    }},
    {"recorder", [] {
        fmt::print(fmt::fg(fmt::color::gold),
            "==================== Loading Recorder ====================\n");
        rmcv_bag::start_recorder_node();
    }},
    {"watchdog", [] {
        fmt::print(fmt::fg(fmt::color::gold),
            "==================== Loading Watchdog ====================\n");
        std::string heartbeat_file = debug::get_session_path() + "/heartbeat";
        watchdog::start_watchdog_node(heartbeat_file, 5000, 1000);
    }},
    {"playback", [] {
        fmt::print(fmt::fg(fmt::color::gold),
            "==================== Loading Playback ====================\n");
        auto bag_path = umt::BasicObjManager<std::string>::find("playback_path");
        if (bag_path) {
            rmcv_bag::start_playback_node(bag_path->get(), 1.0);
        } else {
            debug::print(debug::PrintMode::ERROR, "Playback", "playback_path not set");
        }
    }},
};

// 线程启动后等待函数 (某些线程需要等待就绪)
inline const std::unordered_map<std::string, std::function<void()>> WAIT_MAP = {
    {"param", [] {
        runtime_param::wait_for_param("ok");
        tf::init("camera.yaml");
    }},
    {"hardware", [] {
        // 必须用 find_or_create，否则 find 返回 nullptr 会跳过等待
        auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
        auto app_running = umt::BasicObjManager<bool>::find("app_running");
        while (app_running && !hardware_ready->get() && app_running->get()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }},
};

// 不同模式的线程列表
inline const std::unordered_map<std::string, std::vector<std::string>> MODE_THREADS = {
    {"default",  {"param", "hardware", "detector", "predictor", "recorder", "watchdog"}},
    {"--minimal", {"param", "hardware", "detector"}},
    {"--detector-only", {"param", "hardware", "detector"}},
    {"--no-recorder", {"param", "hardware", "detector", "predictor", "watchdog"}},
    {"--playback", {"param", "playback", "detector", "predictor"}},
};

#endif  // RMCV_MAIN_HPP
