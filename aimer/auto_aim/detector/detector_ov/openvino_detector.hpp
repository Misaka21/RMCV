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
 *   [9-12]:  颜色独热 (red, blue, gray, purple)
 *   [13-21]: 类别独热 (G, 1, 2, 3, 4, 5, O, Bs, Bb)
 */

#ifndef AIMER_AUTOAIM_DETECTOR_OPENVINO_DETECTOR_HPP
#define AIMER_AUTOAIM_DETECTOR_OPENVINO_DETECTOR_HPP

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>

#include "../common/detector_interface.hpp"
#include "../common/types.hpp"

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
 * @brief 基于 OpenVINO 的 YOLO 检测器
 *
 * 特点:
 *   - 使用 Intel OpenVINO 推理引擎
 *   - 支持 CPU 和 GPU (Intel 核显) 推理
 *   - 直接输出关键点，无需后处理角点
 */
class OpenvinoDetector : public DetectorInterface {
public:
    /**
     * @brief 构造函数
     * @param config 检测器配置
     * @param color 敌方颜色
     */
    OpenvinoDetector(const OpenvinoConfig& config, EnemyColor color);

    /**
     * @brief 从配置文件创建检测器
     * @param color 敌方颜色
     * @param config_file 配置文件名
     */
    static std::unique_ptr<OpenvinoDetector> from_config(
        EnemyColor color,
        const std::string& config_file = "detector.toml"
    );

    ~OpenvinoDetector() override;

    // ============================================================================
    // DetectorInterface 接口实现
    // ============================================================================

    std::vector<DetectedArmor> detect(const cv::Mat& image) override;
    void set_enemy_color(EnemyColor color) override { detect_color_ = color; }
    EnemyColor get_enemy_color() const override { return detect_color_; }
    cv::Mat debug_image() const override { return debug_img_; }

private:
    /**
     * @brief 预处理图像
     * @param image 输入图像 (BGR)
     * @return 缩放后的图像和缩放比例
     */
    std::pair<cv::Mat, float> preprocess(const cv::Mat& image);

    /**
     * @brief 后处理推理结果
     * @param output 网络输出张量
     * @param scale 缩放比例 (用于还原坐标)
     * @return 检测到的装甲板列表
     */
    std::vector<DetectedArmor> postprocess(const ov::Tensor& output, float scale);

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

    // OpenVINO 组件
    ov::Core core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;

    // 配置
    OpenvinoConfig config_;
    EnemyColor detect_color_;

    // 调试
    cv::Mat debug_img_;
    std::vector<DetectedArmor> last_detections_;
};

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_OPENVINO_DETECTOR_HPP
