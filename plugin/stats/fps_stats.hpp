//
// FPS统计插件 - 通用帧率和性能统计
//

#ifndef PLUGIN_STATS_FPS_STATS_HPP
#define PLUGIN_STATS_FPS_STATS_HPP

#include <chrono>
#include <functional>
#include <string>

#include "plugin/debug/logger.hpp"

namespace stats {

using SteadyClock = std::chrono::steady_clock;

// 通用FPS统计类
struct FpsStats {
    std::string module_name;
    std::string secondary_label;
    int interval_ms = 10000;  // 输出间隔（毫秒）

    int fps_count = 0;
    int secondary_count = 0;
    float total_latency = 0;
    SteadyClock::time_point last_print_time = SteadyClock::now();

    // 上次计算的结果 (供外部显示用)
    int last_fps = 0;
    int last_secondary = 0;
    float last_avg_latency = 0;

    // 可选的额外信息回调
    std::function<std::string()> extra_info_fn = nullptr;

    // 构造函数
    FpsStats(const std::string& module, const std::string& secondary = "", int interval = 5000)
        : module_name(module), secondary_label(secondary), interval_ms(interval) {}

    // 设置额外信息回调
    void set_extra_info(std::function<std::string()> fn) {
        extra_info_fn = std::move(fn);
    }

    // 更新统计 (每帧调用)
    void tick(float latency_ms = 0, bool secondary_hit = false) {
        fps_count++;
        total_latency += latency_ms;
        if (secondary_hit) {
            secondary_count++;
        }
    }

    // 检查并打印统计 (每帧调用，内部判断是否满间隔)
    void print_if_needed() {
        auto now = SteadyClock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_print_time).count();

        if (elapsed >= interval_ms) {
            print_stats();
            reset();
            last_print_time = now;
        }
    }

    // 更新并打印 (组合调用)
    void update(float latency_ms = 0, bool secondary_hit = false) {
        tick(latency_ms, secondary_hit);
        print_if_needed();
    }

private:
    void print_stats() {
        // 换算为每秒的值
        float scale = 1000.0f / interval_ms;
        last_fps = static_cast<int>(fps_count * scale);
        last_secondary = static_cast<int>(secondary_count * scale);
        last_avg_latency = fps_count > 0 ? total_latency / fps_count : 0;
        std::string extra = extra_info_fn ? ", " + extra_info_fn() : "";

        if (secondary_label.empty()) {
            if (total_latency > 0) {
                debug::print(debug::PrintMode::DEBUG, module_name,
                    "FPS: {}, avg_latency: {:.1f}ms{}", last_fps, last_avg_latency, extra);
            } else {
                debug::print(debug::PrintMode::DEBUG, module_name,
                    "FPS: {}{}", last_fps, extra);
            }
        } else {
            if (total_latency > 0) {
                debug::print(debug::PrintMode::DEBUG, module_name,
                    "FPS: {}, {}: {}, avg_latency: {:.1f}ms{}",
                    last_fps, secondary_label, last_secondary, last_avg_latency, extra);
            } else {
                debug::print(debug::PrintMode::DEBUG, module_name,
                    "FPS: {}, {}: {}{}", last_fps, secondary_label, last_secondary, extra);
            }
        }
    }

    void reset() {
        fps_count = 0;
        secondary_count = 0;
        total_latency = 0;
    }
};

}  // namespace stats

#endif  // PLUGIN_STATS_FPS_STATS_HPP
