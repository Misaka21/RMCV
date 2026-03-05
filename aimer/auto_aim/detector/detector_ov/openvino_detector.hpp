/**
 * @file openvino_detector.hpp
 * @brief 基于 OpenVINO 的 YOLO 装甲板检测器
 *
 * 移植自 RobotPilots 视觉自瞄网络模型
 * 使用 MobileNetV3 backbone 的魔改 YOLOv5
 *
 * 模型输出格式 (每个检测):
 *   [0-7]:   4个关键点坐标 (x0,y0,x1,y1,x2,y2,x3,y3)，从左上角逆时针
 *   [8]:     置信度
 *   [9-12]:  颜色 (red, blue, gray, purple)
 *   [13-21]: 类别 (G, 1, 2, 3, 4, 5, O, Bs, Bb)
 */

#ifndef AIMER_AUTOAIM_DETECTOR_OPENVINO_DETECTOR_HPP
#define AIMER_AUTOAIM_DETECTOR_OPENVINO_DETECTOR_HPP

#include <chrono>
#include <atomic>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>

#include "detector_interface.hpp"

namespace autoaim::detector {

/**
 * @brief OpenVINO 检测器配置
 */
struct OpenvinoConfig {
    std::string model_path;          // ONNX 模型路径
    std::string device = "GPU";      // 推理设备 (CPU/GPU/AUTO)
    int input_size = 640;            // 输入尺寸 (正方形)
    float conf_threshold = 0.65f;    // 置信度阈值
    float nms_threshold = 0.45f;     // NMS 阈值
};

/**
 * @brief 异步推理任务 (内部使用)
 */
struct InferenceTask {
    ov::InferRequest request;
    cv::Mat image;
    float scale;
    int dx, dy;
    int frame_id;
    int64_t timestamp_us;
    serial::SerialReceiveData serial_data;
    std::chrono::steady_clock::time_point submit_time;
};

/**
 * @brief 基于 OpenVINO 的 YOLO 检测器
 *
 * 特点:
 *   - 使用 Intel OpenVINO 推理引擎
 *   - 支持 CPU 和 GPU (Intel 核显) 推理
 *   - 覆盖 push/pop 为真正的异步推理
 *   - 直接输出关键点，无需后处理角点
 */
class OpenvinoDetector : public DetectorInterface {
public:
    OpenvinoDetector(const OpenvinoConfig& config, EnemyColor color);

    static std::unique_ptr<OpenvinoDetector> from_config(
        EnemyColor color,
        const std::string& config_file = "armor_detector.toml"
    );

    ~OpenvinoDetector() override;

    // ========== 同步接口 ==========
    std::vector<DetectedArmor> detect(const cv::Mat& image) override;
    void set_enemy_color(EnemyColor color) override { detect_color_ = color; }
    EnemyColor get_enemy_color() const override { return detect_color_; }
    cv::Mat debug_image() const override { return debug_img_; }
    bool is_async() const override { return true; }  // 真正异步

    // ========== 异步接口 (覆盖基类，真正异步) ==========
    void push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
              const serial::SerialReceiveData& serial_data) override;
    AsyncDetectionResult pop() override;
    size_t queue_size() const override;
    void stop() override;

private:
    std::tuple<cv::Mat, float, int, int> preprocess(const cv::Mat& image);
    std::vector<DetectedArmor> postprocess(const ov::Tensor& output, float scale, int dx, int dy);
    static float sigmoid(float x);
    static ArmorNumber label_to_armor_number(int label);
    static ArmorType get_armor_type(int label, float ratio);

    // OpenVINO 组件
    ov::Core core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;  // 同步模式

    // 异步任务队列 (与基类 result_queue_ 分开)
    mutable std::mutex task_mutex_;
    std::condition_variable task_cv_;
    std::queue<InferenceTask> task_queue_;
    std::atomic<bool> stopped_{false};

    // 配置
    OpenvinoConfig config_;
    EnemyColor detect_color_;

    // 调试
    cv::Mat debug_img_;
    std::vector<DetectedArmor> last_detections_;
};

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_OPENVINO_DETECTOR_HPP
