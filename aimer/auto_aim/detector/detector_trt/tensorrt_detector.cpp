/**
 * @file tensorrt_detector.cpp
 * @brief TensorRT 检测器实现
 */

#include "tensorrt_detector.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <NvOnnxParser.h>

#include "plugin/param/static_config.hpp"

namespace autoaim::detector {

namespace fs = std::filesystem;

// ============================================================================
// 常量定义 (与 OpenVINO 版本相同)
// ============================================================================

namespace output_idx {
    constexpr int LANDMARKS_START = 0;
    constexpr int CONFIDENCE = 8;
    constexpr int COLOR_START = 9;
    constexpr int COLOR_END = 13;
    constexpr int CLASS_START = 13;
    constexpr int CLASS_END = 22;
    constexpr int FEATURE_SIZE = 22;
}

namespace model_color {
    constexpr int RED = 0;
    constexpr int BLUE = 1;
    constexpr int GRAY = 2;
    constexpr int PURPLE = 3;
}

namespace model_class {
    constexpr int SENTRY = 0;
    constexpr int HERO = 1;
    constexpr int ENGINEER = 2;
    constexpr int INFANTRY_3 = 3;
    constexpr int INFANTRY_4 = 4;
    constexpr int INFANTRY_5 = 5;
    constexpr int OUTPOST = 6;
    constexpr int BASE_S = 7;
    constexpr int BASE_B = 8;
}

// ============================================================================
// TrtLogger 实现
// ============================================================================

void TrtLogger::log(Severity severity, const char* msg) noexcept {
    if (severity <= Severity::kWARNING) {
        fmt::print(stderr, "[TensorRT] {}\n", msg);
    }
}

// ============================================================================
// 构造与工厂
// ============================================================================

TensorrtDetector::TensorrtDetector(const TensorrtConfig& config, EnemyColor color)
    : config_(config), detect_color_(color)
{
    // 检查是否有缓存的 engine
    std::string engine_path = get_engine_path();

    if (fs::exists(engine_path)) {
        if (load_engine(engine_path)) {
            fmt::print("[TensorRT] Loaded cached engine: {}\n", engine_path);
        } else {
            fmt::print("[TensorRT] Failed to load cached engine, rebuilding...\n");
            build_engine_from_onnx();
            save_engine(engine_path);
        }
    } else {
        fmt::print("[TensorRT] Building engine from ONNX (this may take a while)...\n");
        build_engine_from_onnx();
        save_engine(engine_path);
    }

    // 获取绑定信息
    input_idx_ = engine_->getBindingIndex("images");
    output_idx_ = engine_->getBindingIndex("output");

    if (input_idx_ < 0 || output_idx_ < 0) {
        // 尝试用索引
        input_idx_ = 0;
        output_idx_ = 1;
    }

    input_dims_ = engine_->getBindingDimensions(input_idx_);
    output_dims_ = engine_->getBindingDimensions(output_idx_);

    // 分配 CUDA 缓冲区
    size_t input_size = 1;
    for (int i = 0; i < input_dims_.nbDims; ++i) {
        input_size *= input_dims_.d[i];
    }

    size_t output_size = 1;
    for (int i = 0; i < output_dims_.nbDims; ++i) {
        output_size *= output_dims_.d[i];
    }

    cudaMalloc(&device_buffers_[input_idx_], input_size * sizeof(float));
    cudaMalloc(&device_buffers_[output_idx_], output_size * sizeof(float));

    output_buffer_.resize(output_size);

    fmt::print("[TensorRT] Engine loaded. Input: {}x{}x{}x{}, Output: {}x{}x{}\n",
               input_dims_.d[0], input_dims_.d[1], input_dims_.d[2], input_dims_.d[3],
               output_dims_.d[0], output_dims_.d[1], output_dims_.d[2]);
}

std::unique_ptr<TensorrtDetector> TensorrtDetector::from_config(
    EnemyColor color,
    const std::string& config_file)
{
    auto param = static_param::parse_file(config_file);

    TensorrtConfig config;
    config.model_path = ASSET_DIR + static_param::get_param<std::string>(
        param, "Detector", "yolo", "model");
    config.input_size = static_cast<int>(static_param::get_param<int64_t>(
        param, "Detector", "yolo", "input_size"));
    config.conf_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector", "yolo", "conf_threshold"));
    config.nms_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector", "yolo", "nms_threshold"));
    config.fp16 = static_param::get_param<bool>(
        param, "Detector", "yolo", "tensorrt", "fp16");
    config.int8 = static_param::get_param<bool>(
        param, "Detector", "yolo", "tensorrt", "int8");
    config.workspace_mb = static_cast<int>(static_param::get_param<int64_t>(
        param, "Detector", "yolo", "tensorrt", "workspace_mb"));

    return std::make_unique<TensorrtDetector>(config, color);
}

TensorrtDetector::~TensorrtDetector() {
    if (device_buffers_[0]) cudaFree(device_buffers_[0]);
    if (device_buffers_[1]) cudaFree(device_buffers_[1]);
}

// ============================================================================
// Engine 构建与加载
// ============================================================================

void TensorrtDetector::build_engine_from_onnx() {
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(
        nvinfer1::createInferBuilder(logger_));
    if (!builder) {
        throw std::runtime_error("Failed to create TensorRT builder");
    }

    // 创建网络 (显式批次)
    const auto explicit_batch = 1U << static_cast<uint32_t>(
        nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
        builder->createNetworkV2(explicit_batch));
    if (!network) {
        throw std::runtime_error("Failed to create network definition");
    }

    // 解析 ONNX
    auto parser = std::unique_ptr<nvonnxparser::IParser>(
        nvonnxparser::createParser(*network, logger_));
    if (!parser->parseFromFile(config_.model_path.c_str(),
                               static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        throw std::runtime_error("Failed to parse ONNX file: " + config_.model_path);
    }

    // 配置构建器
    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(
        builder->createBuilderConfig());

    // 设置工作空间
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                               config_.workspace_mb * (1ULL << 20));

    // FP16 模式
    if (config_.fp16 && builder->platformHasFastFp16()) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        fmt::print("[TensorRT] FP16 mode enabled\n");
    }

    // INT8 模式 (需要校准数据，这里只是启用标志)
    if (config_.int8 && builder->platformHasFastInt8()) {
        config->setFlag(nvinfer1::BuilderFlag::kINT8);
        fmt::print("[TensorRT] INT8 mode enabled (requires calibration data)\n");
    }

    // 构建 engine
    auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
        builder->buildSerializedNetwork(*network, *config));
    if (!serialized) {
        throw std::runtime_error("Failed to build TensorRT engine");
    }

    // 反序列化
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    engine_.reset(runtime_->deserializeCudaEngine(
        serialized->data(), serialized->size()));
    if (!engine_) {
        throw std::runtime_error("Failed to deserialize TensorRT engine");
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_) {
        throw std::runtime_error("Failed to create execution context");
    }
}

bool TensorrtDetector::load_engine(const std::string& engine_path) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file) return false;

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> data(size);
    file.read(data.data(), size);

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));

    if (!engine_) return false;

    context_.reset(engine_->createExecutionContext());
    return context_ != nullptr;
}

void TensorrtDetector::save_engine(const std::string& engine_path) {
    auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
        engine_->serialize());

    std::ofstream file(engine_path, std::ios::binary);
    file.write(static_cast<const char*>(serialized->data()), serialized->size());
    fmt::print("[TensorRT] Engine saved to: {}\n", engine_path);
}

std::string TensorrtDetector::get_engine_path() const {
    // engine 文件名: model_fp16_640.engine 或 model_int8_640.engine
    fs::path onnx_path(config_.model_path);
    std::string precision = config_.int8 ? "int8" : (config_.fp16 ? "fp16" : "fp32");
    std::string engine_name = onnx_path.stem().string() + "_" + precision +
                              "_" + std::to_string(config_.input_size) + ".engine";
    return (onnx_path.parent_path() / engine_name).string();
}

// ============================================================================
// 检测接口
// ============================================================================

std::vector<DetectedArmor> TensorrtDetector::detect(const cv::Mat& image) {
    // 预处理
    auto [resized, scale] = preprocess(image);

    // 转换为 NCHW float 并归一化
    cv::Mat blob;
    cv::dnn::blobFromImage(resized, blob, 1.0/255.0, cv::Size(), cv::Scalar(),
                           true, false, CV_32F);  // RGB, no crop

    // 拷贝到 GPU
    size_t input_size = blob.total() * sizeof(float);
    cudaMemcpy(device_buffers_[input_idx_], blob.ptr<float>(),
               input_size, cudaMemcpyHostToDevice);

    // 推理
    context_->executeV2(device_buffers_);

    // 拷贝输出到 CPU
    size_t output_size = output_buffer_.size() * sizeof(float);
    cudaMemcpy(output_buffer_.data(), device_buffers_[output_idx_],
               output_size, cudaMemcpyDeviceToHost);

    // 后处理
    int num_detections = output_dims_.d[1];
    auto detections = postprocess(output_buffer_.data(), num_detections, scale);

    // 保存调试信息
    last_detections_ = detections;
    debug_img_ = image.clone();
    for (const auto& det : detections) {
        for (size_t i = 0; i < det.landmarks.size(); ++i) {
            cv::circle(debug_img_, det.landmarks[i], 3, cv::Scalar(0, 255, 0), -1);
            if (i > 0) {
                cv::line(debug_img_, det.landmarks[i-1], det.landmarks[i],
                         cv::Scalar(0, 255, 0), 1);
            }
        }
        if (det.landmarks.size() == 4) {
            cv::line(debug_img_, det.landmarks[3], det.landmarks[0],
                     cv::Scalar(0, 255, 0), 1);
        }
        std::string label = armor_number_to_string(det.number) +
                           " " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";
        cv::putText(debug_img_, label, det.center + cv::Point2f(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    }

    return detections;
}

// ============================================================================
// 内部方法
// ============================================================================

std::pair<cv::Mat, float> TensorrtDetector::preprocess(const cv::Mat& image) {
    int target_size = config_.input_size;

    // Letterbox resize
    float scale = std::min(
        static_cast<float>(target_size) / image.cols,
        static_cast<float>(target_size) / image.rows
    );

    int new_w = static_cast<int>(image.cols * scale);
    int new_h = static_cast<int>(image.rows * scale);

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(new_w, new_h));

    // 创建灰色背景
    cv::Mat padded(target_size, target_size, CV_8UC3, cv::Scalar(114, 114, 114));

    // 居中放置
    int dx = (target_size - new_w) / 2;
    int dy = (target_size - new_h) / 2;
    resized.copyTo(padded(cv::Rect(dx, dy, new_w, new_h)));

    return {padded, scale};
}

std::vector<DetectedArmor> TensorrtDetector::postprocess(
    const float* output,
    int num_detections,
    float scale)
{
    int target_color_idx = (detect_color_ == EnemyColor::BLUE)
                           ? model_color::BLUE
                           : model_color::RED;

    std::vector<DetectedArmor> candidates;
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;

    for (int i = 0; i < num_detections; ++i) {
        const float* row = output + i * output_idx::FEATURE_SIZE;

        // 置信度过滤
        float conf = sigmoid(row[output_idx::CONFIDENCE]);
        if (conf < config_.conf_threshold) {
            continue;
        }

        // 解析颜色
        int color = 0;
        float max_color_score = row[output_idx::COLOR_START];
        for (int c = 1; c < 4; ++c) {
            if (row[output_idx::COLOR_START + c] > max_color_score) {
                max_color_score = row[output_idx::COLOR_START + c];
                color = c;
            }
        }

        // 颜色过滤
        if (color == model_color::GRAY || color == model_color::PURPLE) {
            continue;
        }
        if (color != target_color_idx) {
            continue;
        }

        // 解析类别
        int label = 0;
        float max_class_score = row[output_idx::CLASS_START];
        for (int c = 1; c < 9; ++c) {
            if (row[output_idx::CLASS_START + c] > max_class_score) {
                max_class_score = row[output_idx::CLASS_START + c];
                label = c;
            }
        }

        // 解析关键点
        std::vector<cv::Point2f> landmarks(4);
        for (int j = 0; j < 4; ++j) {
            float x = row[output_idx::LANDMARKS_START + j * 2];
            float y = row[output_idx::LANDMARKS_START + j * 2 + 1];
            landmarks[j] = cv::Point2f(x / scale, y / scale);
        }

        // 计算包围盒
        float min_x = landmarks[0].x, max_x = landmarks[0].x;
        float min_y = landmarks[0].y, max_y = landmarks[0].y;
        for (const auto& pt : landmarks) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }
        cv::Rect box(
            static_cast<int>(min_x),
            static_cast<int>(min_y),
            static_cast<int>(max_x - min_x),
            static_cast<int>(max_y - min_y)
        );

        // 计算长宽比
        float length = cv::norm(landmarks[0] - landmarks[3]);
        float width = cv::norm(landmarks[0] - landmarks[1]);
        float ratio = length / std::max(width, 1.0f);

        // 构造检测结果
        DetectedArmor det;
        det.landmarks = landmarks;
        det.center = cv::Point2f(
            (landmarks[0].x + landmarks[2].x) / 2,
            (landmarks[0].y + landmarks[2].y) / 2
        );
        det.confidence = conf;
        det.number = label_to_armor_number(label);
        det.type = get_armor_type(label, ratio);
        det.color = (color == model_color::RED) ? EnemyColor::RED : EnemyColor::BLUE;

        candidates.push_back(det);
        boxes.push_back(box);
        scores.push_back(conf);
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, config_.conf_threshold,
                      config_.nms_threshold, indices);

    std::vector<DetectedArmor> results;
    results.reserve(indices.size());
    for (int idx : indices) {
        results.push_back(candidates[idx]);
    }

    return results;
}

float TensorrtDetector::sigmoid(float x) {
    if (x > 0) {
        return 1.0f / (1.0f + std::exp(-x));
    } else {
        float exp_x = std::exp(x);
        return exp_x / (1.0f + exp_x);
    }
}

ArmorNumber TensorrtDetector::label_to_armor_number(int label) {
    switch (label) {
        case model_class::SENTRY:     return ArmorNumber::SENTRY;
        case model_class::HERO:       return ArmorNumber::HERO;
        case model_class::ENGINEER:   return ArmorNumber::ENGINEER;
        case model_class::INFANTRY_3: return ArmorNumber::INFANTRY_3;
        case model_class::INFANTRY_4: return ArmorNumber::INFANTRY_4;
        case model_class::INFANTRY_5: return ArmorNumber::INFANTRY_5;
        case model_class::OUTPOST:    return ArmorNumber::OUTPOST;
        case model_class::BASE_S:     return ArmorNumber::BASE;
        case model_class::BASE_B:     return ArmorNumber::BASE;
        default:                      return ArmorNumber::UNKNOWN;
    }
}

ArmorType TensorrtDetector::get_armor_type(int label, float ratio) {
    if (label == model_class::HERO || label == model_class::BASE_B) {
        return ArmorType::LARGE;
    }
    if (label == model_class::BASE_S) {
        return ArmorType::SMALL;
    }
    return (ratio > 3.5f) ? ArmorType::LARGE : ArmorType::SMALL;
}

}  // namespace autoaim::detector
