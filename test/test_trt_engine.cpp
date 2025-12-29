/**
 * @brief 测试 TensorRT 检测器初始化 (生成 engine 文件)
 *
 * 用法: ./test_trt_engine
 */

#include <iostream>
#include <opencv2/opencv.hpp>

#include "aimer/auto_aim/detector/detector_trt/tensorrt_detector.hpp"

int main() {
    std::cout << "[INFO] Creating TensorRT detector...\n";
    std::cout << "[INFO] First run will build engine from ONNX (may take several minutes)\n";

    try {
        // 直接构造配置，不依赖配置文件
        autoaim::detector::TensorrtConfig config;
        config.model_path = ASSET_DIR "/armor_0526.onnx";
        config.input_size = 640;
        config.conf_threshold = 0.65f;
        config.nms_threshold = 0.45f;
        config.fp16 = true;
        config.int8 = false;
        config.workspace_mb = 1024;

        std::cout << "[INFO] Model path: " << config.model_path << "\n";

        auto detector = std::make_unique<autoaim::detector::TensorrtDetector>(
            config, autoaim::EnemyColor::BLUE
        );

        std::cout << "[INFO] TensorRT detector created successfully!\n";

        // 测试推理
        cv::Mat test_img(640, 640, CV_8UC3, cv::Scalar(128, 128, 128));
        auto results = detector->detect(test_img);
        std::cout << "[INFO] Test inference: " << results.size() << " detections.\n";

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }

    std::cout << "[INFO] Done! Check asset/ for .engine file.\n";
    return 0;
}
