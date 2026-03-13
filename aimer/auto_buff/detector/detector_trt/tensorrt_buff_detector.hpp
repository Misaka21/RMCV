/**
 * @file tensorrt_buff_detector.hpp
 * @brief 基于 TensorRT 的能量机关 YOLOX 检测器
 *
 * 模型: FYT yolox_rune_3.6m, 输入 480x480
 * 输出格式: [1, N_anchors, 15]
 *
 * 注意: YOLOX 模型不需要 /255 归一化, 因此不使用 CUDA 预处理 kernel,
 *       而是在 CPU 上完成 letterbox + blobFromImage (scale=1.0),
 *       然后 memcpy 到 GPU.
 */

#ifndef AIMER_AUTOBUFF_DETECTOR_TRT_TENSORRT_BUFF_DETECTOR_HPP
#define AIMER_AUTOBUFF_DETECTOR_TRT_TENSORRT_BUFF_DETECTOR_HPP

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <NvInfer.h>
#include <cuda_runtime.h>

#include "aimer/auto_buff/detector/common/detector_interface.hpp"
#include "aimer/auto_buff/detector/common/preprocess.hpp"
#include "aimer/auto_buff/detector/common/postprocess.hpp"
#include "aimer/auto_buff/detector/decoder/rune_decoder.hpp"

namespace autobuff::detector {

/**
 * @brief TensorRT Logger
 */
class TrtBuffLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override;
};

/**
 * @brief TensorRT 检测器配置
 */
struct TrtBuffConfig {
    std::string model_path;         // ONNX 或 engine 文件路径
    int input_size = 480;           // 输入尺寸 (正方形)
    float conf_threshold = 0.50f;   // 置信度阈值
    float nms_threshold  = 0.30f;   // NMS 阈值
    bool fp16 = true;               // 使用 FP16 推理
    bool int8 = false;              // 使用 INT8 量化
    int workspace_mb = 256;         // 工作空间大小 (MB)
};

/**
 * @brief 异步推理资源槽位
 */
struct TrtBuffSlot {
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t stream = nullptr;
    cudaEvent_t  event  = nullptr;
    void* input_device  = nullptr;   // GPU 输入张量 (float32 NCHW)
    void* output_device = nullptr;   // GPU 输出张量
    void* pinned_input  = nullptr;   // 锁页输入缓冲 (CPU blob → H2D)
    void* pinned_output = nullptr;   // 锁页输出缓冲 (D2H)
    bool in_use = false;
};

/**
 * @brief 异步推理任务
 */
struct TrtBuffTask {
    int slot_idx = -1;
    cv::Mat image;
    LetterboxMeta meta;
    int frame_id = 0;
    int64_t timestamp_us = 0;
    serial::SerialReceiveData serial_data;
    std::chrono::steady_clock::time_point submit_time;
};

/**
 * @brief 基于 TensorRT 的能量机关 YOLOX 检测器
 */
class TensorrtBuffDetector : public BuffDetectorInterface {
public:
    TensorrtBuffDetector(const TrtBuffConfig& config, EnemyColor color);

    static std::unique_ptr<TensorrtBuffDetector> from_config(
        EnemyColor color,
        const std::string& config_file = "buff_detector.toml"
    );

    ~TensorrtBuffDetector() override;

    TensorrtBuffDetector(const TensorrtBuffDetector&) = delete;
    TensorrtBuffDetector& operator=(const TensorrtBuffDetector&) = delete;

    // ========== 同步接口 ==========
    BuffDetectionResult detect(const cv::Mat& image, double timestamp) override;
    void set_enemy_color(EnemyColor color) override { color_ = color; }
    EnemyColor get_enemy_color() const override { return color_; }
    cv::Mat get_debug_image() const override { return debug_img_; }
    bool is_async() const override { return true; }

    // ========== 异步接口 ==========
    void push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
              const serial::SerialReceiveData& serial_data) override;
    AsyncBuffDetectionResult pop() override;
    size_t queue_size() const override;
    void stop() override;

private:
    void build_engine_from_onnx();
    bool load_engine(const std::string& engine_path);
    void save_engine(const std::string& engine_path);
    std::string get_engine_path() const;

    void init_async_slots();
    void destroy_async_slots();
    int  acquire_slot();
    void release_slot(int idx);

    // CPU 预处理: letterbox + BGR→RGB + HWC→NCHW (不做 /255 归一化)
    cv::Mat cpu_preprocess(const cv::Mat& image, LetterboxMeta& meta) const;

    TrtBuffConfig  config_;
    EnemyColor     color_;

    // TensorRT 组件
    TrtBuffLogger  logger_;
    std::unique_ptr<nvinfer1::IRuntime>        runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine>     engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;

    // CUDA 缓冲区 (同步模式)
    void* input_device_  = nullptr;
    void* output_device_ = nullptr;
    std::vector<float> output_buffer_;

    cudaStream_t stream_ = nullptr;

    // 模型信息
    std::string input_name_;
    std::string output_name_;
    nvinfer1::Dims input_dims_;
    nvinfer1::Dims output_dims_;
    std::vector<int64_t> output_shape_;
    size_t input_size_  = 0;
    size_t output_size_ = 0;
    int input_binding_idx_  = 0;
    int output_binding_idx_ = 1;

    // 异步推理
    static constexpr int NUM_ASYNC_SLOTS = 2;
    std::array<TrtBuffSlot, NUM_ASYNC_SLOTS> async_slots_;
    mutable std::mutex slot_mutex_;

    mutable std::mutex       task_mutex_;
    std::condition_variable  task_cv_;
    std::queue<TrtBuffTask>  task_queue_;
    std::atomic<bool>        stopped_{false};

    // 解码器和后处理器
    RuneDecoder   decoder_;
    Postprocessor postprocessor_;

    // 调试
    mutable cv::Mat debug_img_;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_TRT_TENSORRT_BUFF_DETECTOR_HPP
