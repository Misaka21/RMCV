#ifndef PLUGIN_WATCHDOG_NODE_HPP
#define PLUGIN_WATCHDOG_NODE_HPP

#include <atomic>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "plugin/debug/logger.hpp"
#include "umt/umt.hpp"

namespace watchdog {

/**
 * @brief 更新节点心跳时间戳
 * @param node_name 节点名称 (hardware, detector, predictor, recorder)
 *
 * 在各节点的主循环中每帧调用一次
 */
inline void heartbeat(const std::string& node_name) {
    auto ts = umt::BasicObjManager<int64_t>::find_or_create("heartbeat_" + node_name, 0);
    ts->get() = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

/**
 * @brief 更新节点数据心跳时间戳
 * @param node_name 节点名称
 *
 * 仅在数据有效时调用（如串口收到数据）
 */
inline void heartbeat_data(const std::string& node_name) {
    auto ts = umt::BasicObjManager<int64_t>::find_or_create("heartbeat_data_" + node_name, 0);
    ts->get() = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

/**
 * @brief 获取节点最后心跳时间戳
 * @param node_name 节点名称
 * @return 最后心跳时间戳(ms)，不存在返回0
 */
inline int64_t get_heartbeat(const std::string& node_name) {
    auto ts = umt::BasicObjManager<int64_t>::find("heartbeat_" + node_name);
    return ts ? ts->get() : 0;
}

/**
 * @brief 获取节点最后数据心跳时间戳
 * @param node_name 节点名称
 * @return 最后数据心跳时间戳(ms)，不存在返回0
 */
inline int64_t get_heartbeat_data(const std::string& node_name) {
    auto ts = umt::BasicObjManager<int64_t>::find("heartbeat_data_" + node_name);
    return ts ? ts->get() : 0;
}

/**
 * @brief 节点配置
 */
struct NodeConfig {
    std::string name;
    bool require_data = false;  // 是否需要检查数据心跳
};

namespace detail {

inline void check_nodes(const std::vector<NodeConfig>& nodes, int timeout_ms) {
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    for (const auto& node : nodes) {
        int64_t last_ts = get_heartbeat(node.name);

        // 首次心跳前不检查（允许长时间初始化，如 TensorRT）
        if (last_ts == 0) continue;

        // 检查 alive 超时
        int64_t elapsed_ms = now_ms - last_ts;
        if (elapsed_ms > timeout_ms) {
            debug::print(debug::PrintMode::FATAL, "WatchdogNode",
                "节点 [{}] 心跳超时: {}ms > {}ms, 进程退出!",
                node.name, elapsed_ms, timeout_ms);
            std::exit(1);
        }

        // 检查 data 心跳（仅 require_data 节点）
        if (node.require_data) {
            int64_t data_ts = get_heartbeat_data(node.name);
            if (data_ts == 0) {
                debug::print(debug::PrintMode::FATAL, "WatchdogNode",
                    "节点 [{}] 数据心跳从未收到, 进程退出!", node.name);
                std::exit(1);
            }
            int64_t data_elapsed = now_ms - data_ts;
            if (data_elapsed > timeout_ms) {
                debug::print(debug::PrintMode::FATAL, "WatchdogNode",
                    "节点 [{}] 数据心跳超时: {}ms > {}ms, 进程退出!",
                    node.name, data_elapsed, timeout_ms);
                std::exit(1);
            }
        }
    }
}

inline void write_heartbeat_file(const std::string& filepath,
                                  const std::vector<NodeConfig>& nodes,
                                  int timeout_ms) {
    if (filepath.empty()) return;

    try {
        std::ofstream ofs(filepath);
        if (!ofs) return;

        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        ofs << "timestamp_ms: " << wall_ms << "\n";
        ofs << "timeout_ms: " << timeout_ms << "\n";
        ofs << "---\n";

        for (const auto& node : nodes) {
            int64_t last_ts = get_heartbeat(node.name);
            if (last_ts == 0) {
                ofs << node.name << ": not started\n";
            } else {
                int64_t elapsed = now_ms - last_ts;
                ofs << node.name << ": " << elapsed << " ms ago";
                if (elapsed > timeout_ms) {
                    ofs << " [TIMEOUT]";
                }

                // 显示 data 心跳（仅 require_data 节点）
                if (node.require_data) {
                    int64_t data_ts = get_heartbeat_data(node.name);
                    if (data_ts == 0) {
                        ofs << " [data: none]";
                    } else {
                        int64_t data_elapsed = now_ms - data_ts;
                        ofs << " [data: " << data_elapsed << " ms ago";
                        if (data_elapsed > timeout_ms) {
                            ofs << " TIMEOUT";
                        }
                        ofs << "]";
                    }
                }
                ofs << "\n";
            }
        }
    } catch (...) {
        // 写文件失败不影响主逻辑
    }
}

} // namespace detail

/**
 * @brief 启动看门狗节点
 *
 * 功能：
 * 1. 监控各工作节点的心跳，超时则 exit(1)
 * 2. 写心跳文件，供外部脚本检测
 *
 * @param heartbeat_file 心跳文件路径
 * @param timeout_ms 超时时间(毫秒)，默认5000ms
 * @param check_interval_ms 检查间隔(毫秒)，默认1000ms
 */
inline void start_watchdog_node(const std::string& heartbeat_file,
                                 int timeout_ms = 5000,
                                 int check_interval_ms = 1000) {
    // 监控的节点列表
    std::vector<NodeConfig> nodes = {
        {"hardware", true},      // 需要检查串口数据心跳
        {"detector", false},     // 只检查存活
        {"predictor", false},    // 只检查存活
        {"fire_control", false}, // 只检查存活
    };

    // 运行标志
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    // 等待节点初始化（减少等待时间）
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));

    debug::print(debug::PrintMode::INFO, "WatchdogNode",
        "启动监控, 超时={}ms, 心跳文件={}", timeout_ms, heartbeat_file);

    while (running->get()) {
        detail::check_nodes(nodes, timeout_ms);
        detail::write_heartbeat_file(heartbeat_file, nodes, timeout_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
    }

    debug::print(debug::PrintMode::INFO, "WatchdogNode", "Watchdog node stopped");
}

} // namespace watchdog

#endif // PLUGIN_WATCHDOG_NODE_HPP
