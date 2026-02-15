// 能量机关检测器抽象接口 (2026)
// - 同步 detect() 或异步 push()/pop()
// - 输出类型: autobuff::BuffDetectionResult

#ifndef AIMER_AUTOBUFF_DETECTOR_INTERFACE_HPP
#define AIMER_AUTOBUFF_DETECTOR_INTERFACE_HPP

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include <opencv2/core.hpp>

#include "aimer/auto_buff/common/types.hpp"
#include "hardware/serial/serial_thread.hpp"

namespace autobuff::detector {

// 从 autobuff 命名空间导入类型
using autobuff::EnemyColor;
using autobuff::DetectionStatus;
using autobuff::DetectedRCenter;
using autobuff::DetectedTarget;
using autobuff::BuffDetectionResult;

/**
 * @brief 异步检测结果
 */
struct AsyncBuffDetectionResult {
    BuffDetectionResult detection;
    cv::Mat image;
    int frame_id = 0;
    int64_t timestamp_us = 0;
    serial::SerialReceiveData serial_data;
    float latency_ms = 0;
};

/**
 * @brief 能量机关检测器抽象接口
 *
 * 所有检测器（传统/YOLO）都必须实现此接口
 *
 * 支持两种使用模式:
 *   1. 同步模式: 直接调用 detect()
 *   2. 异步模式: push() 提交，pop() 获取结果
 */
class BuffDetectorInterface {
public:
    virtual ~BuffDetectorInterface() = default;

    // ========== 同步接口 ==========

    /**
     * @brief 检测能量机关 (同步)
     * @param image 输入图像
     * @param timestamp 时间戳 (秒)
     * @return 检测结果
     */
    virtual BuffDetectionResult detect(const cv::Mat& image, double timestamp) = 0;

    /**
     * @brief 设置敌方颜色
     */
    virtual void set_enemy_color(EnemyColor color) = 0;

    /**
     * @brief 获取当前敌方颜色
     */
    virtual EnemyColor get_enemy_color() const = 0;

    /**
     * @brief 获取调试图像 (可选)
     */
    virtual cv::Mat get_debug_image() const { return {}; }

    /**
     * @brief 是否支持真正的异步推理
     *
     * 返回 true 时，node 使用双线程 push/pop 模式
     * 返回 false 时，node 使用单线程 detect 模式
     */
    virtual bool is_async() const { return false; }

    // ========== 异步接口 (默认实现: 同步模拟) ==========

    /**
     * @brief 异步提交检测 (非阻塞)
     *
     * 默认实现: 同步调用 detect()，结果存入队列
     */
    virtual void push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
                      const serial::SerialReceiveData& serial_data) {
        auto start = std::chrono::steady_clock::now();

        double timestamp_sec = timestamp_us / 1e6;
        auto detection = detect(image, timestamp_sec);

        auto end = std::chrono::steady_clock::now();
        float latency_ms = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / 1000.0f;

        AsyncBuffDetectionResult result;
        result.detection = std::move(detection);
        result.image = image;
        result.frame_id = frame_id;
        result.timestamp_us = timestamp_us;
        result.serial_data = serial_data;
        result.latency_ms = latency_ms;

        {
            std::lock_guard lock(queue_mutex_);
            result_queue_.push(std::move(result));
        }
        queue_cv_.notify_one();
    }

    /**
     * @brief 获取检测结果 (阻塞等待)
     */
    virtual AsyncBuffDetectionResult pop() {
        std::unique_lock lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return !result_queue_.empty(); });
        auto result = std::move(result_queue_.front());
        result_queue_.pop();
        return result;
    }

    /**
     * @brief 获取当前队列长度
     */
    virtual size_t queue_size() const {
        std::lock_guard lock(queue_mutex_);
        return result_queue_.size();
    }

protected:
    // 异步结果队列 (供默认实现和子类使用)
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<AsyncBuffDetectionResult> result_queue_;
};

// 检测器类型枚举
enum class DetectorType {
    TRADITIONAL,  // 传统检测 (轮廓+几何分析)
    YOLO          // YOLO检测 (神经网络关键点)
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_INTERFACE_HPP
