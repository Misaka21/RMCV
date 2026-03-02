/**
 * @file openvino_buff_detector.cpp
 * @brief OpenVINO 能量机关检测器实现
 */

#include "openvino_buff_detector.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

#include "plugin/param/static_config.hpp"
#include "plugin/debug/logger.hpp"

namespace autobuff::detector {

// ============================================================================
// 构造与工厂
// ============================================================================

OpenvinoBuffDetector::OpenvinoBuffDetector(const OvBuffConfig& config, EnemyColor color)
    : config_(config)
    , color_(color)
    , decoder_(config.conf_threshold, config.nms_threshold)
{
    debug::print("info", "OpenVINO-Buff", "Loading model: {}", config_.model_path);

    // 启用模型缓存 (加速后续启动)
    std::string cache_dir = std::string(ASSET_DIR) + "/ov_cache";
    core_.set_property(ov::cache_dir(cache_dir));

    // 读取模型
    model_ = core_.read_model(config_.model_path);

    // 配置预处理
    ov::preprocess::PrePostProcessor ppp(model_);

    // 输入配置: NHWC, BGR, uint8
    ppp.input()
        .tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);

    // 预处理: 转 float32, BGR→RGB, 归一化到 [0,1]
    ppp.input()
        .preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({255.0, 255.0, 255.0});

    // 模型输入布局
    ppp.input().model().set_layout("NCHW");

    // 输出配置
    ppp.output().tensor().set_element_type(ov::element::f32);

    model_ = ppp.build();

    // 编译模型 (THROUGHPUT 模式优化并行推理)
    compiled_model_ = core_.compile_model(
        model_,
        config_.device,
        ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT)
    );

    // 创建推理请求 (同步模式)
    infer_request_ = compiled_model_.create_infer_request();

    debug::print("info", "OpenVINO-Buff",
        "Initialized on device: {} (THROUGHPUT mode)", config_.device);
}

std::unique_ptr<OpenvinoBuffDetector> OpenvinoBuffDetector::from_config(
    EnemyColor color,
    const std::string& config_file)
{
    auto param = static_param::parse_file(config_file);

    OvBuffConfig config;
    config.model_path = std::string(ASSET_DIR) + "/" + static_param::get_param<std::string>(
        param, "Detector.Yolo", "model_path");
    config.input_size = static_cast<int>(static_param::get_param<int64_t>(
        param, "Detector.Yolo", "input_size"));
    config.conf_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector.Yolo", "conf_threshold"));
    config.nms_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector.Yolo", "nms_threshold"));
    config.device = static_param::get_param<std::string>(
        param, "Detector.Yolo.OpenVINO", "device");

    return std::make_unique<OpenvinoBuffDetector>(config, color);
}

OpenvinoBuffDetector::~OpenvinoBuffDetector() = default;

// ============================================================================
// 同步检测接口
// ============================================================================

BuffDetectionResult OpenvinoBuffDetector::detect(const cv::Mat& image, double timestamp)
{
    // 预处理
    auto [letterboxed, meta] = letterbox_resize(image, config_.input_size);

    // 创建输入张量
    ov::Tensor input_tensor(
        compiled_model_.input().get_element_type(),
        compiled_model_.input().get_shape(),
        letterboxed.data
    );

    // 推理
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.infer();

    // 解码输出
    const ov::Tensor& output = infer_request_.get_output_tensor();
    ov::Shape shape = output.get_shape();
    std::vector<int64_t> shape_vec(shape.begin(), shape.end());
    auto raw_objects = decoder_.decode(output.data<float>(), shape_vec, meta);

    // 构建结果 (同步模式，robot_state 由调用方补填)
    aimer::RobotState dummy_state;
    auto result = postprocessor_.build_result(
        raw_objects, meta, dummy_state, 0, timestamp, DetectorBackend::OPENVINO);

    // 保存调试图像
    debug_img_ = image.clone();

    return result;
}

// ============================================================================
// 异步推理接口
// ============================================================================

void OpenvinoBuffDetector::push(const cv::Mat& image, int frame_id,
                                int64_t timestamp_us,
                                const serial::SerialReceiveData& serial_data)
{
    // 队列过长时直接丢弃 (低延迟模式)
    {
        std::lock_guard lock(task_mutex_);
        constexpr size_t MAX_QUEUE_SIZE = 2;
        if (task_queue_.size() >= MAX_QUEUE_SIZE) {
            return;
        }
    }

    // 预处理 (CPU letterbox)
    auto [letterboxed, meta] = letterbox_resize(image, config_.input_size);

    // 每帧创建独立的 InferRequest (支持真正并行)
    auto infer_request = compiled_model_.create_infer_request();

    // 创建输入张量 (张量持有 letterboxed.data 指针，letterboxed 必须随任务存活)
    ov::Tensor input_tensor(
        compiled_model_.input().get_element_type(),
        compiled_model_.input().get_shape(),
        letterboxed.data
    );

    // 异步启动推理
    infer_request.set_input_tensor(input_tensor);
    infer_request.start_async();

    // 入队 (letterboxed 随任务一起存活)
    {
        std::lock_guard lock(task_mutex_);
        task_queue_.push(OvInferenceTask{
            std::move(infer_request),
            image,       // hardware 已 clone
            std::move(letterboxed),
            meta,
            frame_id,
            timestamp_us,
            serial_data,
            std::chrono::steady_clock::now()
        });
    }
    task_cv_.notify_one();
}

AsyncBuffDetectionResult OpenvinoBuffDetector::pop()
{
    OvInferenceTask task;

    // 出队 (阻塞等待)
    {
        std::unique_lock lock(task_mutex_);
        task_cv_.wait(lock, [this] { return !task_queue_.empty(); });
        task = std::move(task_queue_.front());
        task_queue_.pop();
    }

    // 等待推理完成
    task.request.wait();

    // 计算延迟
    auto now = std::chrono::steady_clock::now();
    float latency_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        now - task.submit_time).count() / 1000.0f;

    // 解码输出
    const ov::Tensor& output = task.request.get_output_tensor();
    ov::Shape shape = output.get_shape();
    std::vector<int64_t> shape_vec(shape.begin(), shape.end());
    auto raw_objects = decoder_.decode(output.data<float>(), shape_vec, task.meta);

    // 构建检测结果 (robot_state 由 detector_node 补填)
    aimer::RobotState dummy_state;
    double timestamp_s = task.timestamp_us / 1e6;
    auto detection = postprocessor_.build_result(
        raw_objects, task.meta, dummy_state,
        task.frame_id, timestamp_s, DetectorBackend::OPENVINO);

    // 保存调试图像
    debug_img_ = task.image.clone();

    return AsyncBuffDetectionResult{
        std::move(detection),
        std::move(task.image),
        task.frame_id,
        task.timestamp_us,
        task.serial_data,
        latency_ms
    };
}

size_t OpenvinoBuffDetector::queue_size() const {
    std::lock_guard lock(task_mutex_);
    return task_queue_.size();
}

}  // namespace autobuff::detector
