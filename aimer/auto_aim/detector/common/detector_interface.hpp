//
// 检测器抽象接口
// 所有检测器实现都必须继承此接口
//

#ifndef AIMER_AUTOAIM_DETECTOR_INTERFACE_HPP
#define AIMER_AUTOAIM_DETECTOR_INTERFACE_HPP

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include <opencv2/core.hpp>

#include "aimer/auto_aim/common/types.hpp"
#include "hardware/serial/serial_thread.hpp"

namespace autoaim::detector {

// 从 autoaim 命名空间导入类型
using autoaim::EnemyColor;
using autoaim::ArmorType;
using autoaim::DetectedArmor;

/**
 * @brief 异步检测结果
 */
struct AsyncDetectionResult {
    std::vector<DetectedArmor> armors;
    cv::Mat image;
    int frame_id = 0;
    int64_t timestamp_us = 0;
    serial::SerialReceiveData serial_data;
    float latency_ms = 0;
};

/**
 * @brief 检测器抽象接口
 *
 * 所有检测器（传统/YOLO/...）都必须实现此接口
 *
 * 支持两种使用模式:
 *   1. 同步模式: 直接调用 detect()
 *   2. 异步模式: push() 提交，pop() 获取结果
 *
 * 默认实现: push 内部调用 detect，结果存队列
 * YOLO 检测器覆盖为真正的异步推理
 */
class DetectorInterface {
public:
    virtual ~DetectorInterface() = default;

    // ========== 同步接口 ==========

    /**
     * @brief 检测装甲板 (同步)
     */
    virtual std::vector<DetectedArmor> detect(const cv::Mat& image) = 0;

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
    virtual cv::Mat debug_image() const { return {}; }

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
     * YOLO 检测器覆盖为真正的异步推理
     *
     * @param save_image 是否保存图像用于调试 (false 可节省 ~3ms)
     */
    virtual void push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
                      const serial::SerialReceiveData& serial_data,
                      bool save_image = true) {
        auto start = std::chrono::steady_clock::now();
        auto armors = detect(image);
        auto end = std::chrono::steady_clock::now();

        float latency_ms = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count() / 1000.0f;

        AsyncDetectionResult result;
        result.armors = std::move(armors);
        result.image = save_image ? image.clone() : cv::Mat();
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
    virtual AsyncDetectionResult pop() {
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
    std::queue<AsyncDetectionResult> result_queue_;
};

// 检测器类型枚举
enum class DetectorType {
    TRADITIONAL,  // 传统检测 (灯条匹配)
    YOLO          // YOLO检测 (神经网络)
};

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_INTERFACE_HPP
