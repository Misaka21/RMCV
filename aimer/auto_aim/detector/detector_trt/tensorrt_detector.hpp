/**
 * @file tensorrt_detector.hpp
 * @brief 基于 TensorRT 的 YOLO 装甲板检测器
 *
 * 移植自 RobotPilots 视觉自瞄网络模型
 * 使用 MobileNetV3 backbone 的魔改 YOLOv5
 *
 * 模型输出格式 (每个检测):
 *   [0-7]:   4个关键点坐标 (x0,y0,x1,y1,x2,y2,x3,y3)，从左上角逆时针
 *   [8]:     置信度
 *   [9-12]:  颜色独热 (red, blue, gray, purple)
 *   [13-21]: 类别独热 (G, 1, 2, 3, 4, 5, O, Bs, Bb)
 */

#ifndef AIMER_AUTOAIM_DETECTOR_TENSORRT_DETECTOR_HPP
#define AIMER_AUTOAIM_DETECTOR_TENSORRT_DETECTOR_HPP

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/core.hpp>
#include <NvInfer.h>
#include <cuda_runtime.h>

#include "detector_interface.hpp"
#include "types.hpp"

namespace autoaim::detector {

/**
 * @brief TensorRT Logger
 */
class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override;
};

/**
 * @brief TensorRT 检测器配置
 */
struct TensorrtConfig {
    std::string model_path;          // ONNX 或 engine 文件路径
    int input_size = 640;            // 输入尺寸 (正方形)
    float conf_threshold = 0.65f;    // 置信度阈值
    float nms_threshold = 0.45f;     // NMS 阈值
    bool fp16 = true;                // 使用 FP16 推理
    bool int8 = false;               // 使用 INT8 量化
    int workspace_mb = 1024;         // 工作空间大小 (MB)
};

/**
 * @brief 异步推理资源槽位 (预分配，避免频繁分配)
 */
struct InferenceSlot {
    cudaStream_t stream = nullptr;
    cudaEvent_t event = nullptr;
    void* input_device = nullptr;
    void* output_device = nullptr;
    std::vector<float> output_buffer;
    bool in_use = false;
};

/**
 * @brief 异步推理任务
 */
struct TrtInferenceTask {
    int slot_idx;                    // 使用的槽位索引
    cv::Mat image;                   // 原始图像
    float scale;
    int dx, dy;
    int frame_id;
    int64_t timestamp_us;
    serial::SerialReceiveData serial_data;
    std::chrono::steady_clock::time_point submit_time;
};

/**
 * @brief 基于 TensorRT 的 YOLO 检测器
 *
 * 特点:
 *   - 使用 NVIDIA TensorRT 推理引擎
 *   - 支持 FP16/INT8 量化加速
 *   - 自动缓存编译后的 engine 文件
 *   - 支持异步推理 (push/pop 接口)
 */
class TensorrtDetector : public DetectorInterface {
public:
    /**
     * @brief 构造函数
     * @param config 检测器配置
     * @param color 敌方颜色
     */
    TensorrtDetector(const TensorrtConfig& config, EnemyColor color);

    /**
     * @brief 从配置文件创建检测器
     * @param color 敌方颜色
     * @param config_file 配置文件名
     */
    static std::unique_ptr<TensorrtDetector> from_config(
        EnemyColor color,
        const std::string& config_file = "detector.toml"
    );

    ~TensorrtDetector() override;

    // 禁止拷贝
    TensorrtDetector(const TensorrtDetector&) = delete;
    TensorrtDetector& operator=(const TensorrtDetector&) = delete;

    // ============================================================================
    // DetectorInterface 接口实现
    // ============================================================================

    std::vector<DetectedArmor> detect(const cv::Mat& image) override;
    void set_enemy_color(EnemyColor color) override { detect_color_ = color; }
    EnemyColor get_enemy_color() const override { return detect_color_; }
    cv::Mat debug_image() const override { return debug_img_; }
    bool is_async() const override { return true; }  // 支持异步

    // ============================================================================
    // 异步推理接口 (覆盖基类)
    // ============================================================================

    void push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
              const serial::SerialReceiveData& serial_data) override;
    AsyncDetectionResult pop() override;
    size_t queue_size() const override;

private:
    /**
     * @brief 从 ONNX 构建 TensorRT engine
     */
    void build_engine_from_onnx();

    /**
     * @brief 加载已有的 engine 文件
     */
    bool load_engine(const std::string& engine_path);

    /**
     * @brief 保存 engine 到文件
     */
    void save_engine(const std::string& engine_path);

    /**
     * @brief 获取 engine 缓存路径
     */
    std::string get_engine_path() const;

    /**
     * @brief 预处理图像
     * @param image 输入图像 (BGR)
     * @return {缩放后图像, scale, dx, dy}
     */
    std::tuple<cv::Mat, float, int, int> preprocess(const cv::Mat& image);

    /**
     * @brief 后处理推理结果
     * @param output 网络输出数据
     * @param num_detections 检测数量
     * @param scale 缩放比例
     * @param dx letterbox x 偏移
     * @param dy letterbox y 偏移
     * @return 检测到的装甲板列表
     */
    std::vector<DetectedArmor> postprocess(
        const float* output,
        int num_detections,
        float scale,
        int dx,
        int dy
    );

    /**
     * @brief Sigmoid 函数
     */
    static float sigmoid(float x);

    /**
     * @brief 模型类别索引转 ArmorNumber
     */
    static ArmorNumber label_to_armor_number(int label);

    /**
     * @brief 判断装甲板类型 (大/小)
     */
    static ArmorType get_armor_type(int label, float ratio);

    /**
     * @brief 初始化异步推理资源
     */
    void init_async_slots();

    /**
     * @brief 释放异步推理资源
     */
    void destroy_async_slots();

    /**
     * @brief 获取空闲槽位 (-1 表示无)
     */
    int acquire_slot();

    /**
     * @brief 释放槽位
     */
    void release_slot(int idx);

    // TensorRT 组件
    TrtLogger logger_;
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;

    // CUDA 缓冲区 (同步模式使用)
    void* input_device_ = nullptr;
    void* output_device_ = nullptr;
    std::vector<float> output_buffer_;

    // CUDA 流 (同步模式使用)
    cudaStream_t stream_ = nullptr;

    // 模型信息
    std::string input_name_;
    std::string output_name_;
    nvinfer1::Dims input_dims_;
    nvinfer1::Dims output_dims_;
    size_t input_size_ = 0;
    size_t output_size_ = 0;
    int num_detections_ = 0;

    // Binding 索引 (旧版 API)
    int input_binding_idx_ = 0;
    int output_binding_idx_ = 1;

    // 异步推理资源
    static constexpr int NUM_ASYNC_SLOTS = 2;  // 最多2个并行任务
    std::array<InferenceSlot, NUM_ASYNC_SLOTS> async_slots_;
    mutable std::mutex slot_mutex_;

    // 异步任务队列
    mutable std::mutex task_mutex_;
    std::condition_variable task_cv_;
    std::queue<TrtInferenceTask> task_queue_;

    // 配置
    TensorrtConfig config_;
    EnemyColor detect_color_;

    // 调试
    cv::Mat debug_img_;
    std::vector<DetectedArmor> last_detections_;
};

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_TENSORRT_DETECTOR_HPP
