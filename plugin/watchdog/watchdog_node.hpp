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
 * @brief 获取节点最后心跳时间戳
 * @param node_name 节点名称
 * @return 最后心跳时间戳(ms)，不存在返回0
 */
inline int64_t get_heartbeat(const std::string& node_name) {
    auto ts = umt::BasicObjManager<int64_t>::find("heartbeat_" + node_name);
    return ts ? ts->get() : 0;
}

/**
 * @brief WatchdogNode - 看门狗节点
 *
 * 功能：
 * 1. 监控各工作节点的心跳，超时则 exit(1)
 * 2. 同时写心跳文件，供外部脚本检测 watchdog 自身是否存活
 *
 * 双层保护：
 * - 内层：WatchdogNode 监控 hardware/detector/predictor
 * - 外层：外部脚本检测进程存活 + 心跳文件更新时间
 */
class WatchdogNode {
public:
    /**
     * @brief 启动看门狗节点
     * @param nodes 需要监控的节点名称列表
     * @param timeout_ms 超时时间(毫秒)，默认5000ms
     * @param check_interval_ms 检查间隔(毫秒)，默认1000ms
     * @param heartbeat_file 心跳文件路径，空则不写文件
     */
    void start(const std::vector<std::string>& nodes,
               int timeout_ms = 5000,
               int check_interval_ms = 1000,
               const std::string& heartbeat_file = "/tmp/rmcv_heartbeat") {
        nodes_ = nodes;
        timeout_ms_ = timeout_ms;
        check_interval_ms_ = check_interval_ms;
        heartbeat_file_ = heartbeat_file;
        running_ = true;

        thread_ = std::thread([this]() {
            // 等待所有节点初始化（首次心跳）
            std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms_ * 2));

            debug::print(debug::PrintMode::INFO, "WatchdogNode",
                "启动监控, 超时={}ms, 节点数={}", timeout_ms_, nodes_.size());

            while (running_) {
                check_nodes();
                write_heartbeat_file();
                std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms_));
            }
        });
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    ~WatchdogNode() {
        stop();
    }

private:
    void check_nodes() {
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        for (const auto& node : nodes_) {
            int64_t last_ts = get_heartbeat(node);

            // 节点尚未启动（无心跳记录）
            if (last_ts == 0) {
                continue;
            }

            int64_t elapsed_ms = now_ms - last_ts;

            if (elapsed_ms > timeout_ms_) {
                debug::print(debug::PrintMode::FATAL, "WatchdogNode",
                    "节点 [{}] 心跳超时: {}ms > {}ms, 进程退出!",
                    node, elapsed_ms, timeout_ms_);

                std::exit(1);
            }
        }
    }

    /**
     * @brief 写心跳文件，供外部脚本检测 watchdog 自身是否存活
     *
     * 文件格式:
     *   timestamp_ms: 1704067200000
     *   hardware: 1234 ms ago
     *   detector: 567 ms ago
     *   predictor: 89 ms ago
     */
    void write_heartbeat_file() {
        if (heartbeat_file_.empty()) return;

        try {
            std::ofstream ofs(heartbeat_file_);
            if (!ofs) return;

            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            ofs << "timestamp_ms: " << wall_ms << "\n";
            ofs << "timeout_ms: " << timeout_ms_ << "\n";
            ofs << "---\n";

            for (const auto& node : nodes_) {
                int64_t last_ts = get_heartbeat(node);
                if (last_ts == 0) {
                    ofs << node << ": not started\n";
                } else {
                    int64_t elapsed = now_ms - last_ts;
                    ofs << node << ": " << elapsed << " ms ago";
                    if (elapsed > timeout_ms_) {
                        ofs << " [TIMEOUT]";
                    }
                    ofs << "\n";
                }
            }
        } catch (...) {
            // 写文件失败不影响主逻辑
        }
    }

    std::vector<std::string> nodes_;
    int timeout_ms_ = 5000;
    int check_interval_ms_ = 1000;
    std::string heartbeat_file_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

} // namespace watchdog

#endif // PLUGIN_WATCHDOG_NODE_HPP
