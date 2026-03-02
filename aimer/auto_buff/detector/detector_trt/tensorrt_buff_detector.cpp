/**
 * @file tensorrt_buff_detector.cpp
 * @brief TensorRT 能量机关检测器实现
 */

#include "tensorrt_buff_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <NvOnnxParser.h>
#include <NvInferVersion.h>
#include <cuda_fp16.h>

// 复用 auto_aim 的 CUDA 预处理 kernel
#include "aimer/auto_aim/detector/detector_trt/cuda_preprocess.hpp"

#include "plugin/param/static_config.hpp"
#include "plugin/debug/logger.hpp"
#include "aimer/common/trt_init_mutex.hpp"

namespace autobuff::detector {

namespace fs = std::filesystem;

// ============================================================================
// TrtBuffLogger 实现
// ============================================================================

void TrtBuffLogger::log(Severity severity, const char* msg) noexcept {
    if (severity <= Severity::kWARNING) {
        debug::print("warning", "TensorRT-Buff", "{}", msg);
    }
}

// ============================================================================
// 构造与工厂
// ============================================================================

TensorrtBuffDetector::TensorrtBuffDetector(const TrtBuffConfig& config, EnemyColor color)
    : config_(config)
    , color_(color)
    , decoder_(config.conf_threshold, config.nms_threshold)
{
    // TRT builder 不是线程安全的，串行化所有 TRT 初始化
    std::lock_guard<std::mutex> init_lock(aimer::trt_init_mutex());

    // 阻塞同步模式，避免 CPU 忙等待
    cudaError_t flags_err = cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
    if (flags_err != cudaSuccess && flags_err != cudaErrorSetOnActiveProcess) {
        debug::print("warning", "TensorRT-Buff",
            "Failed to set blocking sync mode: {}", cudaGetErrorString(flags_err));
    }

    debug::print("info", "TensorRT-Buff", "Loading model: {}", config_.model_path);

    std::string engine_path = get_engine_path();
    debug::print("info", "TensorRT-Buff", "Looking for engine: {}", engine_path);

    if (fs::exists(engine_path)) {
        if (load_engine(engine_path)) {
            debug::print("info", "TensorRT-Buff", "Loaded cached engine: {}", engine_path);
        } else {
            debug::print("warning", "TensorRT-Buff", "Failed to load cached engine, rebuilding...");
            build_engine_from_onnx();
            save_engine(engine_path);
        }
    } else {
        debug::print("info", "TensorRT-Buff", "Building engine from ONNX (this may take a while)...");
        build_engine_from_onnx();
        save_engine(engine_path);
    }

    // 创建 CUDA 流
    cudaError_t stream_err = cudaStreamCreate(&stream_);
    if (stream_err != cudaSuccess) {
        throw std::runtime_error("Failed to create CUDA stream: " +
                                 std::string(cudaGetErrorString(stream_err)));
    }

    // 解析 binding 信息 (旧版 API，兼容 TRT 8.2+)
    int num_bindings = engine_->getNbBindings();
    debug::print("info", "TensorRT-Buff", "Number of bindings: {}", num_bindings);

    for (int i = 0; i < num_bindings; ++i) {
        const char* name   = engine_->getBindingName(i);
        bool is_input      = engine_->bindingIsInput(i);
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

        debug::print("info", "TensorRT-Buff",
            "  Binding {}: name='{}', type={}, is_input={}, dims=[{},{},{},{}]",
            i, name, dtype_str, is_input, dims.d[0], dims.d[1], dims.d[2],
            dims.nbDims > 3 ? dims.d[3] : 0);

        if (is_input) {
            input_name_        = name;
            input_dims_        = dims;
            input_binding_idx_ = i;
            use_fp16_input_    = (dtype == nvinfer1::DataType::kHALF);
        } else {
            output_name_        = name;
            output_dims_        = dims;
            output_binding_idx_ = i;
        }
    }

    if (input_name_.empty() || output_name_.empty()) {
        throw std::runtime_error("Failed to find input/output tensors");
    }

    // 计算缓冲区大小
    input_size_ = 1;
    for (int i = 0; i < input_dims_.nbDims; ++i) {
        input_size_ *= static_cast<size_t>(input_dims_.d[i]);
    }
    output_size_ = 1;
    for (int i = 0; i < output_dims_.nbDims; ++i) {
        output_size_ *= static_cast<size_t>(output_dims_.d[i]);
    }

    size_t input_elem_size = use_fp16_input_ ? sizeof(__half) : sizeof(float);

    debug::print("info", "TensorRT-Buff",
        "Buffer sizes: input={} elements ({}KB, {}), output={} floats ({}KB)",
        input_size_, input_size_ * input_elem_size / 1024,
        use_fp16_input_ ? "FP16" : "FP32",
        output_size_, output_size_ * sizeof(float) / 1024);

    // 预分配原图缓冲区 (最大 1920x1200)
    img_buffer_size_ = 1920 * 1200 * 3;
    cudaError_t err0 = cudaMalloc(&img_device_,    img_buffer_size_);
    cudaError_t err1 = cudaMalloc(&input_device_,  input_size_ * input_elem_size);
    cudaError_t err2 = cudaMalloc(&output_device_, output_size_ * sizeof(float));

    if (err0 != cudaSuccess || err1 != cudaSuccess || err2 != cudaSuccess) {
        throw std::runtime_error("Failed to allocate CUDA memory");
    }

    output_buffer_.resize(output_size_);

    // 缓存 output shape (避免每帧分配)
    output_shape_.reserve(output_dims_.nbDims);
    for (int i = 0; i < output_dims_.nbDims; ++i) {
        output_shape_.push_back(output_dims_.d[i]);
    }

    debug::print("info", "TensorRT-Buff",
        "Engine loaded. Input: '{}' (binding {}), Output: '{}' (binding {})",
        input_name_, input_binding_idx_, output_name_, output_binding_idx_);

    // Warmup
    debug::print("info", "TensorRT-Buff", "Warming up...");
    void* bindings[2];
    bindings[input_binding_idx_]  = input_device_;
    bindings[output_binding_idx_] = output_device_;
    for (int i = 0; i < 3; ++i) {
        context_->enqueueV2(bindings, stream_, nullptr);
        cudaStreamSynchronize(stream_);
    }
    debug::print("info", "TensorRT-Buff", "Warmup done");

    init_async_slots();
}

std::unique_ptr<TensorrtBuffDetector> TensorrtBuffDetector::from_config(
    EnemyColor color,
    const std::string& config_file)
{
    auto param = static_param::parse_file(config_file);

    TrtBuffConfig config;
    config.model_path = std::string(ASSET_DIR) + "/" + static_param::get_param<std::string>(
        param, "Detector.Yolo", "model_path");
    config.input_size = static_cast<int>(static_param::get_param<int64_t>(
        param, "Detector.Yolo", "input_size"));
    config.conf_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector.Yolo", "conf_threshold"));
    config.nms_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector.Yolo", "nms_threshold"));
    config.fp16 = static_param::get_param<bool>(
        param, "Detector.Yolo.TensorRT", "fp16");
    config.int8 = static_param::get_param<bool>(
        param, "Detector.Yolo.TensorRT", "int8");
    config.workspace_mb = static_cast<int>(static_param::get_param<int64_t>(
        param, "Detector.Yolo.TensorRT", "workspace_mb"));

    return std::make_unique<TensorrtBuffDetector>(config, color);
}

TensorrtBuffDetector::~TensorrtBuffDetector() {
    destroy_async_slots();
    if (stream_)        cudaStreamDestroy(stream_);
    if (img_device_)    cudaFree(img_device_);
    if (input_device_)  cudaFree(input_device_);
    if (output_device_) cudaFree(output_device_);
}

// ============================================================================
// Engine 构建与加载
// ============================================================================

void TensorrtBuffDetector::build_engine_from_onnx() {
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(
        nvinfer1::createInferBuilder(logger_));
    if (!builder) {
        throw std::runtime_error("Failed to create TensorRT builder");
    }

    const auto explicit_batch = 1U << static_cast<uint32_t>(
        nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
        builder->createNetworkV2(explicit_batch));

    auto parser = std::unique_ptr<nvonnxparser::IParser>(
        nvonnxparser::createParser(*network, logger_));
    if (!parser->parseFromFile(config_.model_path.c_str(),
                               static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        throw std::runtime_error("Failed to parse ONNX: " + config_.model_path);
    }

    auto build_config = std::unique_ptr<nvinfer1::IBuilderConfig>(
        builder->createBuilderConfig());

#if NV_TENSORRT_MAJOR > 8 || (NV_TENSORRT_MAJOR == 8 && NV_TENSORRT_MINOR >= 4)
    build_config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                                     config_.workspace_mb * (1ULL << 20));
#else
    build_config->setMaxWorkspaceSize(config_.workspace_mb * (1ULL << 20));
#endif

    if (config_.fp16 && builder->platformHasFastFp16()) {
        build_config->setFlag(nvinfer1::BuilderFlag::kFP16);
        debug::print("info", "TensorRT-Buff", "FP16 mode enabled");
    }

    auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
        builder->buildSerializedNetwork(*network, *build_config));
    if (!serialized) {
        throw std::runtime_error("Failed to build TensorRT engine");
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    engine_.reset(runtime_->deserializeCudaEngine(serialized->data(), serialized->size()));
    if (!engine_) {
        throw std::runtime_error("Failed to deserialize engine");
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_) {
        throw std::runtime_error("Failed to create execution context");
    }
}

bool TensorrtBuffDetector::load_engine(const std::string& engine_path) {
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

void TensorrtBuffDetector::save_engine(const std::string& engine_path) {
    auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(engine_->serialize());
    std::ofstream file(engine_path, std::ios::binary);
    file.write(static_cast<const char*>(serialized->data()), serialized->size());
    debug::print("info", "TensorRT-Buff", "Engine saved to: {}", engine_path);
}

std::string TensorrtBuffDetector::get_engine_path() const {
    fs::path onnx_path(config_.model_path);
    std::string precision = config_.int8 ? "int8" : (config_.fp16 ? "fp16" : "fp32");
    std::string name = onnx_path.stem().string() + "_" + precision +
                       "_" + std::to_string(config_.input_size) + ".engine";
    return (onnx_path.parent_path() / name).string();
}

// ============================================================================
// 同步检测接口
// ============================================================================

BuffDetectionResult TensorrtBuffDetector::detect(const cv::Mat& image, double timestamp)
{
    cv::Mat continuous = image.isContinuous() ? image : image.clone();
    size_t img_size = continuous.total() * continuous.elemSize();
    if (img_size > img_buffer_size_) {
        debug::print("error", "TensorRT-Buff",
            "Image too large: {} > {}", img_size, img_buffer_size_);
        return {};
    }

    // 计算 letterbox 参数（GPU kernel 自行完成 letterbox，这里只需 meta）
    float scale = std::min(
        static_cast<float>(config_.input_size) / continuous.cols,
        static_cast<float>(config_.input_size) / continuous.rows
    );
    int new_w = static_cast<int>(continuous.cols * scale);
    int new_h = static_cast<int>(continuous.rows * scale);
    LetterboxMeta meta;
    meta.src_w  = continuous.cols;
    meta.src_h  = continuous.rows;
    meta.net_w  = config_.input_size;
    meta.net_h  = config_.input_size;
    meta.scale  = scale;
    meta.pad_x  = static_cast<float>((config_.input_size - new_w) / 2);
    meta.pad_y  = static_cast<float>((config_.input_size - new_h) / 2);

    cudaMemcpyAsync(img_device_, continuous.data, img_size,
                    cudaMemcpyHostToDevice, stream_);

    float gpu_scale;
    int gpu_dx, gpu_dy;
    if (use_fp16_input_) {
        autoaim::detector::cuda_preprocess_fp16(
            static_cast<uint8_t*>(img_device_),
            static_cast<__half*>(input_device_),
            continuous.cols, continuous.rows,
            config_.input_size, &gpu_scale, &gpu_dx, &gpu_dy, stream_, false);
    } else {
        autoaim::detector::cuda_preprocess(
            static_cast<uint8_t*>(img_device_),
            static_cast<float*>(input_device_),
            continuous.cols, continuous.rows,
            config_.input_size, &gpu_scale, &gpu_dx, &gpu_dy, stream_, false);
    }

    void* bindings[2];
    bindings[input_binding_idx_]  = input_device_;
    bindings[output_binding_idx_] = output_device_;
    context_->enqueueV2(bindings, stream_, nullptr);

    cudaMemcpyAsync(output_buffer_.data(), output_device_,
                    output_size_ * sizeof(float), cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);

    // 构建 shape (解码器需要)
    auto raw_objects = decoder_.decode(output_buffer_.data(), output_shape_, meta);

    aimer::RobotState dummy_state;
    auto result = postprocessor_.build_result(
        raw_objects, meta, dummy_state, 0, timestamp, DetectorBackend::TENSORRT);

    debug_img_ = image.clone();
    return result;
}

// ============================================================================
// 异步推理实现
// ============================================================================

void TensorrtBuffDetector::init_async_slots() {
    size_t input_elem_size = use_fp16_input_ ? sizeof(__half) : sizeof(float);

    for (int i = 0; i < NUM_ASYNC_SLOTS; ++i) {
        auto& slot = async_slots_[i];

        slot.context = engine_->createExecutionContext();
        if (!slot.context) {
            throw std::runtime_error(
                "Failed to create context for slot " + std::to_string(i));
        }

        cudaStreamCreate(&slot.stream);
        cudaEventCreateWithFlags(&slot.event, cudaEventBlockingSync);
        cudaMalloc(&slot.img_device,    img_buffer_size_);
        cudaMalloc(&slot.input_device,  input_size_ * input_elem_size);
        cudaMalloc(&slot.output_device, output_size_ * sizeof(float));

        // 锁页内存暂存区：让 H2D 变成真正的异步 DMA
        cudaMallocHost(&slot.pinned_buffer, img_buffer_size_);
        // 锁页输出缓冲：让 D2H 变成真正的异步 DMA
        cudaMallocHost(&slot.pinned_output, output_size_ * sizeof(float));

        slot.in_use = false;
    }
    debug::print("info", "TensorRT-Buff",
        "Initialized {} async slots ({})", NUM_ASYNC_SLOTS,
        use_fp16_input_ ? "FP16" : "FP32");
}

void TensorrtBuffDetector::destroy_async_slots() {
    for (int i = 0; i < NUM_ASYNC_SLOTS; ++i) {
        auto& slot = async_slots_[i];
        if (slot.stream) {
            cudaStreamSynchronize(slot.stream);
            cudaStreamDestroy(slot.stream);
        }
        if (slot.event)         cudaEventDestroy(slot.event);
        if (slot.img_device)    cudaFree(slot.img_device);
        if (slot.input_device)  cudaFree(slot.input_device);
        if (slot.output_device) cudaFree(slot.output_device);
        if (slot.pinned_buffer) cudaFreeHost(slot.pinned_buffer);
        if (slot.pinned_output) cudaFreeHost(slot.pinned_output);
        if (slot.context) {
            delete slot.context;
            slot.context = nullptr;
        }
    }
}

int TensorrtBuffDetector::acquire_slot() {
    std::lock_guard lock(slot_mutex_);
    for (int i = 0; i < NUM_ASYNC_SLOTS; ++i) {
        if (!async_slots_[i].in_use) {
            async_slots_[i].in_use = true;
            return i;
        }
    }
    return -1;
}

void TensorrtBuffDetector::release_slot(int idx) {
    if (idx >= 0 && idx < NUM_ASYNC_SLOTS) {
        std::lock_guard lock(slot_mutex_);
        async_slots_[idx].in_use = false;
    }
}

void TensorrtBuffDetector::push(const cv::Mat& image, int frame_id,
                                int64_t timestamp_us,
                                const serial::SerialReceiveData& serial_data)
{
    auto submit_time = std::chrono::steady_clock::now();

    {
        std::lock_guard lock(task_mutex_);
        constexpr size_t MAX_QUEUE_SIZE = 2;
        if (task_queue_.size() >= MAX_QUEUE_SIZE) {
            return;
        }
    }

    int slot_idx = acquire_slot();
    if (slot_idx < 0) {
        return;
    }

    auto& slot = async_slots_[slot_idx];

    cv::Mat continuous = image.isContinuous() ? image : image.clone();
    size_t img_size = continuous.total() * continuous.elemSize();

    // 计算 meta (同步版)
    float scale = std::min(
        static_cast<float>(config_.input_size) / continuous.cols,
        static_cast<float>(config_.input_size) / continuous.rows
    );
    int new_w = static_cast<int>(continuous.cols * scale);
    int new_h = static_cast<int>(continuous.rows * scale);
    LetterboxMeta meta;
    meta.src_w  = continuous.cols;
    meta.src_h  = continuous.rows;
    meta.net_w  = config_.input_size;
    meta.net_h  = config_.input_size;
    meta.scale  = scale;
    meta.pad_x  = static_cast<float>((config_.input_size - new_w) / 2);
    meta.pad_y  = static_cast<float>((config_.input_size - new_h) / 2);

    // 拷贝到锁页内存暂存区，然后异步 DMA 到 GPU
    std::memcpy(slot.pinned_buffer, continuous.data, img_size);
    cudaMemcpyAsync(slot.img_device, slot.pinned_buffer, img_size,
                    cudaMemcpyHostToDevice, slot.stream);

    float gpu_scale;
    int gpu_dx, gpu_dy;
    if (use_fp16_input_) {
        autoaim::detector::cuda_preprocess_fp16(
            static_cast<uint8_t*>(slot.img_device),
            static_cast<__half*>(slot.input_device),
            continuous.cols, continuous.rows,
            config_.input_size, &gpu_scale, &gpu_dx, &gpu_dy, slot.stream, false);
    } else {
        autoaim::detector::cuda_preprocess(
            static_cast<uint8_t*>(slot.img_device),
            static_cast<float*>(slot.input_device),
            continuous.cols, continuous.rows,
            config_.input_size, &gpu_scale, &gpu_dx, &gpu_dy, slot.stream, false);
    }

    void* bindings[2];
    bindings[input_binding_idx_]  = slot.input_device;
    bindings[output_binding_idx_] = slot.output_device;
    slot.context->enqueueV2(bindings, slot.stream, nullptr);

    cudaMemcpyAsync(slot.pinned_output, slot.output_device,
                    output_size_ * sizeof(float), cudaMemcpyDeviceToHost, slot.stream);
    cudaEventRecord(slot.event, slot.stream);

    {
        std::lock_guard lock(task_mutex_);
        task_queue_.push(TrtBuffTask{
            slot_idx, image, meta,
            frame_id, timestamp_us, serial_data, submit_time
        });
    }
    task_cv_.notify_one();
}

AsyncBuffDetectionResult TensorrtBuffDetector::pop()
{
    TrtBuffTask task;

    {
        std::unique_lock lock(task_mutex_);
        task_cv_.wait(lock, [this] { return !task_queue_.empty() || stopped_.load(); });
        if (stopped_.load() && task_queue_.empty()) {
            return {};
        }
        task = std::move(task_queue_.front());
        task_queue_.pop();
    }

    auto& slot = async_slots_[task.slot_idx];
    cudaEventSynchronize(slot.event);

    auto now = std::chrono::steady_clock::now();
    float latency_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        now - task.submit_time).count() / 1000.0f;

    auto raw_objects = decoder_.decode(
        static_cast<const float*>(slot.pinned_output), output_shape_, task.meta);

    aimer::RobotState dummy_state;
    double timestamp_s = task.timestamp_us / 1e6;
    auto detection = postprocessor_.build_result(
        raw_objects, task.meta, dummy_state,
        task.frame_id, timestamp_s, DetectorBackend::TENSORRT);

    release_slot(task.slot_idx);
    debug_img_ = task.image;  // 浅拷贝，按需 clone 由调用方决定

    return AsyncBuffDetectionResult{
        std::move(detection),
        std::move(task.image),
        task.frame_id,
        task.timestamp_us,
        task.serial_data,
        latency_ms
    };
}

size_t TensorrtBuffDetector::queue_size() const {
    std::lock_guard lock(task_mutex_);
    return task_queue_.size();
}

void TensorrtBuffDetector::stop() {
    stopped_.store(true);
    task_cv_.notify_all();
}

}  // namespace autobuff::detector
