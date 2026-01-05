/**
 * @file tensorrt_detector.cpp
 * @brief TensorRT 检测器实现
 */

#include "tensorrt_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <filesystem>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <NvOnnxParser.h>

#include "plugin/param/static_config.hpp"
#include "plugin/debug/logger.hpp"

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

// 模型颜色索引 (模型输出顺序: red, blue, gray, purple)
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
        debug::print("warning", "TensorRT", "{}", msg);
    }
}

// ============================================================================
// 构造与工厂
// ============================================================================

TensorrtDetector::TensorrtDetector(const TensorrtConfig& config, EnemyColor color)
    : config_(config), detect_color_(color)
{
    debug::print("info", "TensorRT", "Loading model: {}", config_.model_path);

    // 检查是否有缓存的 engine
    std::string engine_path = get_engine_path();

    if (fs::exists(engine_path)) {
        if (load_engine(engine_path)) {
            debug::print("info", "TensorRT", "Loaded cached engine: {}", engine_path);
        } else {
            debug::print("warning", "TensorRT", "Failed to load cached engine, rebuilding...");
            build_engine_from_onnx();
            save_engine(engine_path);
        }
    } else {
        debug::print("info", "TensorRT", "Building engine from ONNX (this may take a while)...");
        build_engine_from_onnx();
        save_engine(engine_path);
    }

    // 创建 CUDA 流
    cudaError_t stream_err = cudaStreamCreate(&stream_);
    if (stream_err != cudaSuccess) {
        throw std::runtime_error("Failed to create CUDA stream: " +
                                std::string(cudaGetErrorString(stream_err)));
    }

    // 使用旧版 API 获取 binding 信息 (兼容 TensorRT 8.2+)
    int num_bindings = engine_->getNbBindings();
    debug::print("info", "TensorRT", "Number of bindings: {}", num_bindings);

    for (int i = 0; i < num_bindings; ++i) {
        const char* name = engine_->getBindingName(i);
        bool is_input = engine_->bindingIsInput(i);
        nvinfer1::Dims dims = engine_->getBindingDimensions(i);

        debug::print("info", "TensorRT", "  Binding {}: name='{}', is_input={}, nbDims={}, dims=[{},{},{},{}]",
            i, name, is_input, dims.nbDims, dims.d[0], dims.d[1], dims.d[2],
            dims.nbDims > 3 ? dims.d[3] : 0);

        if (is_input) {
            input_name_ = name;
            input_dims_ = dims;
            input_binding_idx_ = i;
        } else {
            output_name_ = name;
            output_dims_ = dims;
            output_binding_idx_ = i;
        }
    }

    if (input_name_.empty() || output_name_.empty()) {
        throw std::runtime_error("Failed to find input/output tensors");
    }

    // 计算缓冲区大小
    input_size_ = 1;
    for (int i = 0; i < input_dims_.nbDims; ++i) {
        input_size_ *= input_dims_.d[i];
    }

    output_size_ = 1;
    for (int i = 0; i < output_dims_.nbDims; ++i) {
        output_size_ *= output_dims_.d[i];
    }

    // 获取检测数量
    num_detections_ = output_dims_.d[1];

    debug::print("info", "TensorRT", "Buffer sizes: input={} floats ({}KB), output={} floats ({}KB)",
        input_size_, input_size_ * 4 / 1024, output_size_, output_size_ * 4 / 1024);

    // 分配 CUDA 缓冲区
    cudaError_t err1 = cudaMalloc(&input_device_, input_size_ * sizeof(float));
    cudaError_t err2 = cudaMalloc(&output_device_, output_size_ * sizeof(float));

    if (err1 != cudaSuccess || err2 != cudaSuccess) {
        throw std::runtime_error("Failed to allocate CUDA memory: input=" +
                                std::string(cudaGetErrorString(err1)) + ", output=" +
                                std::string(cudaGetErrorString(err2)));
    }

    debug::print("info", "TensorRT", "cudaMalloc: input_ptr={}, output_ptr={}",
        input_device_, output_device_);

    output_buffer_.resize(output_size_);

    debug::print("info", "TensorRT", "Engine loaded. Input: '{}' [{}x{}x{}x{}] (binding {}), Output: '{}' [{}x{}x{}] (binding {})",
               input_name_, input_dims_.d[0], input_dims_.d[1], input_dims_.d[2], input_dims_.d[3], input_binding_idx_,
               output_name_, output_dims_.d[0], output_dims_.d[1], output_dims_.d[2], output_binding_idx_);

    // 检查输出维度顺序
    if (output_dims_.d[1] == 22 && output_dims_.d[2] == 25200) {
        debug::print("warning", "TensorRT", "Output is transposed! [1,22,25200] instead of [1,25200,22]");
    }

    // Warmup (预热GPU，提高首帧速度)
    debug::print("info", "TensorRT", "Warming up...");
    void* bindings[2];
    bindings[input_binding_idx_] = input_device_;
    bindings[output_binding_idx_] = output_device_;
    for (int i = 0; i < 3; ++i) {
        context_->enqueueV2(bindings, stream_, nullptr);
        cudaStreamSynchronize(stream_);
    }
    debug::print("info", "TensorRT", "Warmup done");

    // 初始化异步推理资源
    init_async_slots();
}

std::unique_ptr<TensorrtDetector> TensorrtDetector::from_config(
    EnemyColor color,
    const std::string& config_file)
{
    auto param = static_param::parse_file(config_file);

    TensorrtConfig config;
    config.model_path = std::string(ASSET_DIR) + "/" + static_param::get_param<std::string>(
        param, "Detector.yolo", "model");
    config.input_size = static_cast<int>(static_param::get_param<int64_t>(
        param, "Detector.yolo", "input_size"));
    config.conf_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector.yolo", "conf_threshold"));
    config.nms_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector.yolo", "nms_threshold"));
    config.fp16 = static_param::get_param<bool>(
        param, "Detector.yolo.tensorrt", "fp16");
    config.int8 = static_param::get_param<bool>(
        param, "Detector.yolo.tensorrt", "int8");
    config.workspace_mb = static_cast<int>(static_param::get_param<int64_t>(
        param, "Detector.yolo.tensorrt", "workspace_mb"));

    return std::make_unique<TensorrtDetector>(config, color);
}

TensorrtDetector::~TensorrtDetector() {
    // 释放异步推理资源
    destroy_async_slots();

    // 释放同步模式资源
    if (stream_) cudaStreamDestroy(stream_);
    if (input_device_) cudaFree(input_device_);
    if (output_device_) cudaFree(output_device_);
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
        debug::print("info", "TensorRT", "FP16 mode enabled");
    }

    // INT8 模式 (需要校准数据，这里只是启用标志)
    if (config_.int8 && builder->platformHasFastInt8()) {
        config->setFlag(nvinfer1::BuilderFlag::kINT8);
        debug::print("info", "TensorRT", "INT8 mode enabled (requires calibration data)");
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
    debug::print("info", "TensorRT", "Engine saved to: {}", engine_path);
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
    auto [resized, scale, dx, dy] = preprocess(image);

    // DEBUG: 检查预处理后的图像
    static int preprocess_debug = 0;
    if (++preprocess_debug <= 3) {
        cv::Scalar mean_val = cv::mean(resized);
        debug::print("info", "TensorRT", "Preprocess: input={}x{}, resized={}x{}, scale={:.3f}, dx={}, dy={}, mean=[{:.1f},{:.1f},{:.1f}]",
            image.cols, image.rows, resized.cols, resized.rows, scale, dx, dy,
            mean_val[0], mean_val[1], mean_val[2]);
    }

    // 转换为 NCHW float 并归一化
    cv::Mat blob;
    cv::dnn::blobFromImage(resized, blob, 1.0/255.0, cv::Size(), cv::Scalar(),
                           true, false, CV_32F);  // RGB, no crop

    // DEBUG: 检查输入数据（统计整个 blob）
    static int input_debug = 0;
    if (++input_debug <= 3) {
        float* ptr = blob.ptr<float>();
        int total = blob.total();
        float sum = 0, min_val = ptr[0], max_val = ptr[0];
        for (int i = 0; i < total; ++i) {
            sum += ptr[i];
            min_val = std::min(min_val, ptr[i]);
            max_val = std::max(max_val, ptr[i]);
        }
        debug::print("info", "TensorRT", "Input blob: total={}, sum={:.0f}, min={:.4f}, max={:.4f}, mean={:.4f}",
            total, sum, min_val, max_val, sum / total);
    }

    // 拷贝输入到 GPU
    cudaError_t err = cudaMemcpy(input_device_, blob.ptr<float>(),
               input_size_ * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        debug::print("error", "TensorRT", "cudaMemcpy to device failed: {}", cudaGetErrorString(err));
    }

    // 构建 bindings 数组 (旧版 API)
    void* bindings[2];
    bindings[input_binding_idx_] = input_device_;
    bindings[output_binding_idx_] = output_device_;

    // 推理 (使用旧版 API, 兼容 TensorRT 8.2+)
    bool success = context_->enqueueV2(bindings, stream_, nullptr);

    // 同步等待推理完成
    cudaStreamSynchronize(stream_);

    // 拷贝输出到 CPU (同步)
    cudaError_t copy_err = cudaMemcpy(output_buffer_.data(), output_device_,
               output_size_ * sizeof(float), cudaMemcpyDeviceToHost);
    if (copy_err != cudaSuccess) {
        debug::print("error", "TensorRT", "cudaMemcpy from device failed: {}", cudaGetErrorString(copy_err));
    }

    // 同步等待完成
    cudaError_t sync_err = cudaStreamSynchronize(stream_);

    static int exec_debug = 0;
    if (++exec_debug <= 5) {
        debug::print("info", "TensorRT", "enqueueV2: success={}, sync_err={}",
            success, cudaGetErrorString(sync_err));

        // 输出前20个值来检查数据
        std::string first_vals = "";
        for (int i = 0; i < 20 && i < static_cast<int>(output_buffer_.size()); ++i) {
            first_vals += fmt::format("{:.2f},", output_buffer_[i]);
        }
        debug::print("info", "TensorRT", "First 20 output values: {}", first_vals);

        // 统计 NaN 数量
        int nan_count = 0;
        int inf_count = 0;
        float min_val = output_buffer_[0], max_val = output_buffer_[0];
        for (size_t i = 0; i < output_buffer_.size(); ++i) {
            if (std::isnan(output_buffer_[i])) nan_count++;
            else if (std::isinf(output_buffer_[i])) inf_count++;
            else {
                min_val = std::min(min_val, output_buffer_[i]);
                max_val = std::max(max_val, output_buffer_[i]);
            }
        }
        debug::print("info", "TensorRT", "Output stats: nan={}, inf={}, min={:.2f}, max={:.2f}",
            nan_count, inf_count, min_val, max_val);
    }

    // 后处理
    // DEBUG: 检查原始输出数据
    static int debug_frame = 0;
    if (++debug_frame <= 3) {  // 只打印前3帧
        // 找置信度最高的检测
        int best_idx = 0;
        float best_conf = output_buffer_[8];
        for (int i = 1; i < num_detections_; ++i) {
            float conf = output_buffer_[i * 22 + 8];
            if (conf > best_conf) {
                best_conf = conf;
                best_idx = i;
            }
        }
        const float* best = output_buffer_.data() + best_idx * 22;
        debug::print("info", "TensorRT",
            "Best detection [{}]: landmarks=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}], conf_raw={:.2f}, sigmoid={:.4f}",
            best_idx,
            best[0], best[1], best[2], best[3], best[4], best[5], best[6], best[7],
            best[8], sigmoid(best[8]));
    }

    auto detections = postprocess(output_buffer_.data(), num_detections_, scale, dx, dy);

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

std::tuple<cv::Mat, float, int, int> TensorrtDetector::preprocess(const cv::Mat& image) {
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

    return {padded, scale, dx, dy};
}

std::vector<DetectedArmor> TensorrtDetector::postprocess(
    const float* output,
    int num_detections,
    float scale,
    int dx,
    int dy)
{
    // detect_color_ 是串口传来的 enemy_color (敌方颜色)
    // enemy_color=RED(敌方红) → 检测红色装甲板
    // enemy_color=BLUE(敌方蓝) → 检测蓝色装甲板
    int target_color_idx = (detect_color_ == EnemyColor::RED)
                           ? model_color::RED
                           : model_color::BLUE;

    std::vector<DetectedArmor> candidates;
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;

    // DEBUG: 统计信息
    float max_conf = 0;
    int above_threshold = 0;
    int color_filtered = 0;

    for (int i = 0; i < num_detections; ++i) {
        const float* row = output + i * output_idx::FEATURE_SIZE;

        // 置信度过滤
        float conf = sigmoid(row[output_idx::CONFIDENCE]);
        max_conf = std::max(max_conf, conf);

        if (conf < config_.conf_threshold) {
            continue;
        }
        above_threshold++;

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
        // - purple 丢弃 (无意义)
        // - gray 保留 (灯条闪烁时会短暂变灰，传给 predictor 做消抖)
        // - 敌方颜色保留
        if (color == model_color::PURPLE) {
            color_filtered++;
            continue;
        }
        if (color != target_color_idx && color != model_color::GRAY) {
            color_filtered++;
            continue;  // 既不是敌方颜色，也不是灰色
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

        // 解析关键点 (还原 letterbox 坐标)
        std::vector<cv::Point2f> landmarks(4);
        for (int j = 0; j < 4; ++j) {
            float x = row[output_idx::LANDMARKS_START + j * 2];
            float y = row[output_idx::LANDMARKS_START + j * 2 + 1];
            // 减去 padding 偏移，再除以缩放比例
            landmarks[j] = cv::Point2f((x - dx) / scale, (y - dy) / scale);
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
        // 设置颜色 (灰色用 GRAY 表示，供 predictor 消抖)
        if (color == model_color::GRAY) {
            det.color = EnemyColor::GRAY;
        } else {
            det.color = (color == model_color::RED) ? EnemyColor::RED : EnemyColor::BLUE;
        }

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

    // DEBUG: 输出统计信息
    // static int frame_count = 0;
    // if (++frame_count % 30 == 0) {  // 每30帧输出一次
    //     debug::print("debug", "TensorRT",
    //         "detections={}, max_conf={:.3f}, above_thresh={}, color_filtered={}, final={}",
    //         num_detections, max_conf, above_threshold, color_filtered, results.size());
    // }

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

// ============================================================================
// 异步推理实现
// ============================================================================

void TensorrtDetector::init_async_slots() {
    for (int i = 0; i < NUM_ASYNC_SLOTS; ++i) {
        auto& slot = async_slots_[i];

        // 创建独立的CUDA流
        cudaStreamCreate(&slot.stream);

        // 创建事件用于同步
        cudaEventCreate(&slot.event);

        // 分配GPU缓冲区
        cudaMalloc(&slot.input_device, input_size_ * sizeof(float));
        cudaMalloc(&slot.output_device, output_size_ * sizeof(float));

        // 分配CPU输出缓冲区
        slot.output_buffer.resize(output_size_);

        slot.in_use = false;
    }
    debug::print("info", "TensorRT", "Initialized {} async inference slots", NUM_ASYNC_SLOTS);
}

void TensorrtDetector::destroy_async_slots() {
    for (int i = 0; i < NUM_ASYNC_SLOTS; ++i) {
        auto& slot = async_slots_[i];

        if (slot.stream) {
            cudaStreamSynchronize(slot.stream);
            cudaStreamDestroy(slot.stream);
        }
        if (slot.event) cudaEventDestroy(slot.event);
        if (slot.input_device) cudaFree(slot.input_device);
        if (slot.output_device) cudaFree(slot.output_device);
    }
}

int TensorrtDetector::acquire_slot() {
    std::lock_guard lock(slot_mutex_);
    for (int i = 0; i < NUM_ASYNC_SLOTS; ++i) {
        if (!async_slots_[i].in_use) {
            async_slots_[i].in_use = true;
            return i;
        }
    }
    return -1;  // 无空闲槽位
}

void TensorrtDetector::release_slot(int idx) {
    if (idx >= 0 && idx < NUM_ASYNC_SLOTS) {
        std::lock_guard lock(slot_mutex_);
        async_slots_[idx].in_use = false;
    }
}

void TensorrtDetector::push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
                            const serial::SerialReceiveData& serial_data)
{
    // 检查队列是否已满
    {
        std::lock_guard lock(task_mutex_);
        constexpr size_t MAX_QUEUE_SIZE = 2;
        if (task_queue_.size() >= MAX_QUEUE_SIZE) {
            return;  // 丢弃当前帧
        }
    }

    // 获取空闲槽位
    int slot_idx = acquire_slot();
    if (slot_idx < 0) {
        return;  // 无可用槽位，丢弃
    }

    auto& slot = async_slots_[slot_idx];

    // 预处理
    auto [resized, scale, dx, dy] = preprocess(image);

    // 转换为 NCHW float 并归一化
    cv::Mat blob;
    cv::dnn::blobFromImage(resized, blob, 1.0/255.0, cv::Size(), cv::Scalar(),
                           true, false, CV_32F);

    // 异步拷贝输入到 GPU
    cudaMemcpyAsync(slot.input_device, blob.ptr<float>(),
                    input_size_ * sizeof(float), cudaMemcpyHostToDevice, slot.stream);

    // 构建 bindings 数组
    void* bindings[2];
    bindings[input_binding_idx_] = slot.input_device;
    bindings[output_binding_idx_] = slot.output_device;

    // 异步推理
    context_->enqueueV2(bindings, slot.stream, nullptr);

    // 异步拷贝输出到 CPU
    cudaMemcpyAsync(slot.output_buffer.data(), slot.output_device,
                    output_size_ * sizeof(float), cudaMemcpyDeviceToHost, slot.stream);

    // 记录事件 (用于检测完成)
    cudaEventRecord(slot.event, slot.stream);

    // 入队任务
    {
        std::lock_guard lock(task_mutex_);
        task_queue_.push(TrtInferenceTask{
            slot_idx,
            image.clone(),
            scale, dx, dy,
            frame_id, timestamp_us,
            serial_data,
            std::chrono::steady_clock::now()
        });
    }
    task_cv_.notify_one();
}

AsyncDetectionResult TensorrtDetector::pop()
{
    TrtInferenceTask task;

    // 出队
    {
        std::unique_lock lock(task_mutex_);
        task_cv_.wait(lock, [this] { return !task_queue_.empty(); });
        task = std::move(task_queue_.front());
        task_queue_.pop();
    }

    auto& slot = async_slots_[task.slot_idx];

    // 等待推理完成
    cudaEventSynchronize(slot.event);

    // 计算延迟
    auto now = std::chrono::steady_clock::now();
    float latency_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        now - task.submit_time).count() / 1000.0f;

    // 后处理
    auto armors = postprocess(slot.output_buffer.data(), num_detections_,
                              task.scale, task.dx, task.dy);

    // 释放槽位
    release_slot(task.slot_idx);

    return AsyncDetectionResult{
        std::move(armors),
        std::move(task.image),
        task.frame_id,
        task.timestamp_us,
        task.serial_data,
        latency_ms
    };
}

size_t TensorrtDetector::queue_size() const {
    std::lock_guard lock(task_mutex_);
    return task_queue_.size();
}

}  // namespace autoaim::detector
