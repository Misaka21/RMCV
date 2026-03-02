/**
 * @file openvino_buff_detector.hpp
 * @brief 基于 OpenVINO 的能量机关 YOLO 检测器
 *
 * 模型: sp25 单类别扇叶检测模型
 * 输出格式: [1, 17, 8400]
 *   17 = 4 box(cx,cy,w,h) + 1 score + 6*2 keypoints
 */

#ifndef AIMER_AUTOBUFF_DETECTOR_OV_OPENVINO_BUFF_DETECTOR_HPP
#define AIMER_AUTOBUFF_DETECTOR_OV_OPENVINO_BUFF_DETECTOR_HPP

#include <chrono>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>

#include "aimer/auto_buff/detector/common/detector_interface.hpp"
#include "aimer/auto_buff/detector/common/preprocess.hpp"
#include "aimer/auto_buff/detector/common/postprocess.hpp"
#include "aimer/auto_buff/detector/decoder/buff_decoder.hpp"

namespace autobuff::detector {

/**
 * @brief OpenVINO 检测器配置
 */
struct OvBuffConfig {
    std::string model_path;           // ONNX 模型路径
    std::string device = "GPU";       // 推理设备 (CPU/GPU/AUTO)
    int input_size = 640;             // 输入尺寸 (正方形)
    float conf_threshold = 0.45f;     // 置信度阈值
    float nms_threshold  = 0.45f;     // NMS 阈值
};

/**
 * @brief 异步推理任务 (内部使用)
 */
struct OvInferenceTask {
    ov::InferRequest request;
    cv::Mat image;
    cv::Mat letterboxed;   // 保持有效直到推理完成
    LetterboxMeta meta;
    int frame_id = 0;
    int64_t timestamp_us = 0;
    serial::SerialReceiveData serial_data;
    std::chrono::steady_clock::time_point submit_time;
};

/**
 * @brief 基于 OpenVINO 的能量机关 YOLO 检测器
 *
 * 特点:
 *   - 使用 Intel OpenVINO 推理引擎
 *   - 支持 CPU 和 GPU (Intel 核显) 推理
 *   - 覆盖 push/pop 为真正的异步推理
 *   - 直接输出扇叶关键点，通过后处理计算 R 标和槽位
 */
class OpenvinoBuffDetector : public BuffDetectorInterface {
public:
    OpenvinoBuffDetector(const OvBuffConfig& config, EnemyColor color);

    /**
     * @brief 从 buff_detector.toml 配置文件创建检测器
     */
    static std::unique_ptr<OpenvinoBuffDetector> from_config(
        EnemyColor color,
        const std::string& config_file = "buff_detector.toml"
    );

    ~OpenvinoBuffDetector() override;

    // ========== 同步接口 ==========
    BuffDetectionResult detect(const cv::Mat& image, double timestamp) override;
    void set_enemy_color(EnemyColor color) override { color_ = color; }
    EnemyColor get_enemy_color() const override { return color_; }
    cv::Mat get_debug_image() const override { return debug_img_; }
    bool is_async() const override { return true; }

    // ========== 异步接口 (真正异步) ==========
    void push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
              const serial::SerialReceiveData& serial_data) override;
    AsyncBuffDetectionResult pop() override;
    size_t queue_size() const override;

private:
    OvBuffConfig config_;
    EnemyColor   color_;

    // OpenVINO 组件
    ov::Core          core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest  infer_request_;  // 同步模式

    // 解码器和后处理器
    Sp25Decoder  decoder_;
    Postprocessor postprocessor_;

    // 异步任务队列
    mutable std::mutex       task_mutex_;
    std::condition_variable  task_cv_;
    std::queue<OvInferenceTask> task_queue_;

    // 调试
    mutable cv::Mat debug_img_;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_OV_OPENVINO_BUFF_DETECTOR_HPP
