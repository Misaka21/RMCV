//
// 能量机关检测器节点
// 连接 Hardware 和 Predictor 的中间层
//

#ifndef AUTOBUFF_DETECTOR_NODE_HPP
#define AUTOBUFF_DETECTOR_NODE_HPP

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "common/detector_interface.hpp"
#include "detector_factory.hpp"

namespace autobuff::detector {

/**
 * @brief 检测器节点配置
 */
struct BuffDetectorNodeConfig {
    std::string config_file = "buff.toml";
    bool async_mode = false;  // 是否使用异步模式
};

/**
 * @brief 能量机关检测器节点
 *
 * 负责:
 * 1. 从 Hardware 获取图像帧
 * 2. 调用检测器进行检测
 * 3. 发布检测结果到 UMT
 */
class BuffDetectorNode {
public:
    explicit BuffDetectorNode(const BuffDetectorNodeConfig& config = {});
    ~BuffDetectorNode();

    /**
     * @brief 启动检测器节点
     */
    void start();

    /**
     * @brief 停止检测器节点
     */
    void stop();

    /**
     * @brief 是否正在运行
     */
    bool is_running() const { return running_.load(); }

    /**
     * @brief 获取检测器引用 (用于调试)
     */
    BuffDetectorInterface* get_detector() { return detector_.get(); }

private:
    /**
     * @brief 检测线程主循环
     */
    void detection_loop();

    /**
     * @brief 处理单帧 (同步模式)
     */
    void process_frame_sync();

    /**
     * @brief 处理单帧 (异步模式)
     */
    void process_frame_async();

    BuffDetectorNodeConfig config_;
    std::unique_ptr<BuffDetectorInterface> detector_;

    std::thread detection_thread_;
    std::atomic<bool> running_{false};

    // 统计
    int frame_count_ = 0;
    double total_latency_ms_ = 0;
};

/**
 * @brief 后台启动检测器节点
 * @param config_file 配置文件名
 *
 * 用于 Python 绑定或异步启动
 */
void background_buff_detector_run(const std::string& config_file = "buff.toml");

}  // namespace autobuff::detector

#endif  // AUTOBUFF_DETECTOR_NODE_HPP
