/**
 * @file tensorrt_detector.cpp
 * @brief TensorRT 检测器实现
 */

#include "tensorrt_detector.hpp"
#include "cuda_preprocess.hpp"
#include "int8_calibrator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <filesystem>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <NvOnnxParser.h>
#include <NvInferVersion.h>
#include <cuda_fp16.h>

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
    // 设置 CUDA 调度模式为阻塞同步，避免 CPU 忙等待
    // 注意：必须在第一个 CUDA API 调用之前设置
    cudaError_t flags_err = cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
    if (flags_err != cudaSuccess && flags_err != cudaErrorSetOnActiveProcess) {
        debug::print("warning", "TensorRT", "Failed to set blocking sync mode: {}",
                     cudaGetErrorString(flags_err));
    }

    debug::print("info", "TensorRT", "Loading model: {}", config_.model_path);

    // 检查是否有缓存的 engine
    std::string engine_path = get_engine_path();
    debug::print("info", "TensorRT", "Looking for engine: {}", engine_path);

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
        nvinfer1::DataType dtype = engine_->getBindingDataType(i);

        const char* dtype_str = "unknown";
        switch (dtype) {
            case nvinfer1::DataType::kFLOAT: dtype_str = "FP32"; break;
            case nvinfer1::DataType::kHALF:  dtype_str = "FP16"; break;
            case nvinfer1::DataType::kINT8:  dtype_str = "INT8"; break;
            case nvinfer1::DataType::kINT32: dtype_str = "INT32"; break;
            default: break;
        }

        debug::print("info", "TensorRT", "  Binding {}: name='{}', type={}, is_input={}, dims=[{},{},{},{}]",
            i, name, dtype_str, is_input, dims.d[0], dims.d[1], dims.d[2],
            dims.nbDims > 3 ? dims.d[3] : 0);

        if (is_input) {
            input_name_ = name;
            input_dims_ = dims;
            input_binding_idx_ = i;
            // 检测 FP16 输入
            use_fp16_input_ = (dtype == nvinfer1::DataType::kHALF);
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

    // 计算输入元素大小
    size_t input_elem_size = use_fp16_input_ ? sizeof(__half) : sizeof(float);

    debug::print("info", "TensorRT", "Buffer sizes: input={} elements ({}KB, {}), output={} floats ({}KB)",
        input_size_, input_size_ * input_elem_size / 1024, use_fp16_input_ ? "FP16" : "FP32",
        output_size_, output_size_ * 4 / 1024);

    // 分配 CUDA 缓冲区
    // 预分配原图缓冲区 (假设最大 1920x1200 图像)
    img_buffer_size_ = 1920 * 1200 * 3;  // BGR uint8
    cudaError_t err0 = cudaMalloc(&img_device_, img_buffer_size_);
    cudaError_t err1 = cudaMalloc(&input_device_, input_size_ * input_elem_size);
    cudaError_t err2 = cudaMalloc(&output_device_, output_size_ * sizeof(float));

    if (err0 != cudaSuccess || err1 != cudaSuccess || err2 != cudaSuccess) {
        throw std::runtime_error("Failed to allocate CUDA memory: img=" +
                                std::string(cudaGetErrorString(err0)) + ", input=" +
                                std::string(cudaGetErrorString(err1)) + ", output=" +
                                std::string(cudaGetErrorString(err2)));
    }

    debug::print("info", "TensorRT", "cudaMalloc: img_ptr={}, input_ptr={}, output_ptr={}",
        img_device_, input_device_, output_device_);

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
    if (img_device_) cudaFree(img_device_);
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

    // 设置工作空间 (TensorRT 8.4+ 使用新 API)
#if NV_TENSORRT_MAJOR > 8 || (NV_TENSORRT_MAJOR == 8 && NV_TENSORRT_MINOR >= 4)
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                               config_.workspace_mb * (1ULL << 20));
#else
    config->setMaxWorkspaceSize(config_.workspace_mb * (1ULL << 20));
#endif

    // FP16 模式
    if (config_.fp16 && builder->platformHasFastFp16()) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        debug::print("info", "TensorRT", "FP16 mode enabled");
    }

    // INT8 模式 (需要校准数据)
    std::unique_ptr<Int8EntropyCalibrator> calibrator;
    if (config_.int8 && builder->platformHasFastInt8()) {
        config->setFlag(nvinfer1::BuilderFlag::kINT8);

        // 校准图片目录和缓存文件路径
        fs::path model_dir = fs::path(config_.model_path).parent_path();
        std::string calib_images_dir = (model_dir / "int8_calib_data").string();
        std::string calib_cache_file = (model_dir / "int8_calib.cache").string();

        // 创建校准器 (会自动加载缓存或从图片校准)
        calibrator = std::make_unique<Int8EntropyCalibrator>(
            config_.input_size, calib_images_dir, calib_cache_file);
        config->setInt8Calibrator(calibrator.get());

        debug::print("info", "TensorRT", "INT8 mode enabled with calibration");
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
    // 确保图像是连续的
    cv::Mat continuous_image = image.isContinuous() ? image : image.clone();

    // 检查图像大小是否超过缓冲区
    size_t img_size = continuous_image.total() * continuous_image.elemSize();
    if (img_size > img_buffer_size_) {
        debug::print("error", "TensorRT", "Image size {} exceeds buffer size {}", img_size, img_buffer_size_);
        return {};
    }

    // 拷贝原图到 GPU
    cudaMemcpyAsync(img_device_, continuous_image.data, img_size, cudaMemcpyHostToDevice, stream_);

    // GPU 预处理: letterbox + BGR→RGB + normalize + HWC→CHW
    float scale;
    int dx, dy;
    if (use_fp16_input_) {
        cuda_preprocess_fp16(
            static_cast<uint8_t*>(img_device_),
            static_cast<__half*>(input_device_),
            image.cols, image.rows,
            config_.input_size,
            &scale, &dx, &dy,
            stream_,
            false  // 使用最近邻，速度更快
        );
    } else {
        cuda_preprocess(
            static_cast<uint8_t*>(img_device_),
            static_cast<float*>(input_device_),
            image.cols, image.rows,
            config_.input_size,
            &scale, &dx, &dy,
            stream_,
            false  // 使用最近邻，速度更快
        );
    }

    // 构建 bindings 数组 (旧版 API)
    void* bindings[2];
    bindings[input_binding_idx_] = input_device_;
    bindings[output_binding_idx_] = output_device_;

    // 推理 (使用旧版 API, 兼容 TensorRT 8.2+)
    context_->enqueueV2(bindings, stream_, nullptr);

    // 拷贝输出到 CPU
    cudaMemcpyAsync(output_buffer_.data(), output_device_,
                    output_size_ * sizeof(float), cudaMemcpyDeviceToHost, stream_);

    // 同步等待完成
    cudaStreamSynchronize(stream_);

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

    // 预计算 logit 阈值: sigmoid(x) > thresh  <=>  x > log(thresh/(1-thresh))
    // 避免对 25200 个检测都调用 sigmoid
    const float logit_threshold = std::log(config_.conf_threshold / (1.0f - config_.conf_threshold));

    for (int i = 0; i < num_detections; ++i) {
        const float* row = output + i * output_idx::FEATURE_SIZE;

        // 快速过滤：用 logit 阈值 (避免 sigmoid 计算)
        float raw_conf = row[output_idx::CONFIDENCE];
        if (raw_conf < logit_threshold) {
            continue;
        }

        // 通过初筛后才计算 sigmoid
        float conf = sigmoid(raw_conf);

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
            continue;
        }
        if (color != target_color_idx && color != model_color::GRAY) {
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
    size_t input_elem_size = use_fp16_input_ ? sizeof(__half) : sizeof(float);

    for (int i = 0; i < NUM_ASYNC_SLOTS; ++i) {
        auto& slot = async_slots_[i];

        // 每个槽位创建独立的 ExecutionContext (关键!)
        // 这样多个槽位可以真正并行推理
        slot.context = engine_->createExecutionContext();
        if (!slot.context) {
            throw std::runtime_error("Failed to create execution context for slot " + std::to_string(i));
        }

        // 创建独立的CUDA流
        cudaStreamCreate(&slot.stream);

        // 创建事件用于同步 (使用 BlockingSync 避免 CPU 忙等待)
        cudaEventCreateWithFlags(&slot.event, cudaEventBlockingSync);

        // 分配GPU缓冲区
        cudaMalloc(&slot.img_device, img_buffer_size_);  // 原图缓冲
        cudaMalloc(&slot.input_device, input_size_ * input_elem_size);  // FP16 或 FP32
        cudaMalloc(&slot.output_device, output_size_ * sizeof(float));

        // 分配CPU输出缓冲区
        slot.output_buffer.resize(output_size_);

        slot.in_use = false;
    }
    debug::print("info", "TensorRT", "Initialized {} async slots (input: {})",
        NUM_ASYNC_SLOTS, use_fp16_input_ ? "FP16" : "FP32");
}

void TensorrtDetector::destroy_async_slots() {
    for (int i = 0; i < NUM_ASYNC_SLOTS; ++i) {
        auto& slot = async_slots_[i];

        if (slot.stream) {
            cudaStreamSynchronize(slot.stream);
            cudaStreamDestroy(slot.stream);
        }
        if (slot.event) cudaEventDestroy(slot.event);
        if (slot.img_device) cudaFree(slot.img_device);
        if (slot.input_device) cudaFree(slot.input_device);
        if (slot.output_device) cudaFree(slot.output_device);
        if (slot.context) {
            delete slot.context;  // 销毁独立的 context
            slot.context = nullptr;
        }
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
    // 记录提交时间 (用于计算端到端延迟)
    auto submit_time = std::chrono::steady_clock::now();

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

    // 确保图像是连续的
    cv::Mat continuous_image = image.isContinuous() ? image : image.clone();
    size_t img_size = continuous_image.total() * continuous_image.elemSize();

    // 异步拷贝原图到 GPU
    cudaMemcpyAsync(slot.img_device, continuous_image.data, img_size,
                    cudaMemcpyHostToDevice, slot.stream);

    // GPU 预处理: letterbox + BGR→RGB + normalize + HWC→CHW
    float scale;
    int dx, dy;
    if (use_fp16_input_) {
        cuda_preprocess_fp16(
            static_cast<uint8_t*>(slot.img_device),
            static_cast<__half*>(slot.input_device),
            image.cols, image.rows,
            config_.input_size,
            &scale, &dx, &dy,
            slot.stream,
            false  // 使用最近邻
        );
    } else {
        cuda_preprocess(
            static_cast<uint8_t*>(slot.img_device),
            static_cast<float*>(slot.input_device),
            image.cols, image.rows,
            config_.input_size,
            &scale, &dx, &dy,
            slot.stream,
            false  // 使用最近邻
        );
    }

    // 构建 bindings 数组
    void* bindings[2];
    bindings[input_binding_idx_] = slot.input_device;
    bindings[output_binding_idx_] = slot.output_device;

    // 异步推理 (使用槽位独立的 context，实现真正并行!)
    slot.context->enqueueV2(bindings, slot.stream, nullptr);

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
            image,  // Hardware 已 clone，直接赋值
            scale, dx, dy,
            frame_id, timestamp_us,
            serial_data,
            submit_time
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
