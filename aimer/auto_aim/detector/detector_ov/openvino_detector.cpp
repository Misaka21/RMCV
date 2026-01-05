/**
 * @file openvino_detector.cpp
 * @brief OpenVINO 检测器实现
 */

#include "openvino_detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "plugin/param/static_config.hpp"
#include "plugin/debug/logger.hpp"

namespace autoaim::detector {

// ============================================================================
// 常量定义
// ============================================================================

// 模型输出列索引
namespace output_idx {
    constexpr int LANDMARKS_START = 0;   // 关键点起始 (0-7: x0,y0,x1,y1,x2,y2,x3,y3)
    constexpr int CONFIDENCE = 8;        // 置信度
    constexpr int COLOR_START = 9;       // 颜色起始 (9-12: red,blue,gray,purple)
    constexpr int COLOR_END = 13;
    constexpr int CLASS_START = 13;      // 类别起始 (13-21: G,1,2,3,4,5,O,Bs,Bb)
    constexpr int CLASS_END = 22;
}

// 模型颜色索引 (模型输出顺序: red, blue, gray, purple)
namespace model_color {
    constexpr int RED = 0;
    constexpr int BLUE = 1;
    constexpr int GRAY = 2;
    constexpr int PURPLE = 3;
}

// 模型类别索引 (G, 1, 2, 3, 4, 5, O, Bs, Bb)
namespace model_class {
    constexpr int SENTRY = 0;      // G (哨兵)
    constexpr int HERO = 1;        // 1 (英雄)
    constexpr int ENGINEER = 2;    // 2 (工程)
    constexpr int INFANTRY_3 = 3;  // 3
    constexpr int INFANTRY_4 = 4;  // 4
    constexpr int INFANTRY_5 = 5;  // 5
    constexpr int OUTPOST = 6;     // O (前哨站)
    constexpr int BASE_S = 7;      // Bs (基地小)
    constexpr int BASE_B = 8;      // Bb (基地大)
}

// ============================================================================
// 构造与工厂
// ============================================================================

OpenvinoDetector::OpenvinoDetector(const OpenvinoConfig& config, EnemyColor color)
    : config_(config), detect_color_(color)
{
    debug::print("info", "OpenVINO", "Loading model: {}", config_.model_path);

    // 启用模型缓存 (加速后续启动)
    std::string cache_dir = std::string(ASSET_DIR) + "/ov_cache";
    core_.set_property(ov::cache_dir(cache_dir));
    debug::print("info", "OpenVINO", "Cache directory: {}", cache_dir);

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

    // 构建模型
    model_ = ppp.build();

    // 编译模型
    // THROUGHPUT 模式高吞吐但 CPU 占用高，限制线程数来平衡
    compiled_model_ = core_.compile_model(
        model_,
        config_.device,
        ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT),
        ov::inference_num_threads(4),  // 限制 CPU 线程数
        ov::num_streams(2)             // 限制并行推理流
    );

    // 创建推理请求 (用于同步模式)
    infer_request_ = compiled_model_.create_infer_request();

    debug::print("info", "OpenVINO", "Detector initialized on device: {} (THROUGHPUT, 4 threads, 2 streams)", config_.device);
}

std::unique_ptr<OpenvinoDetector> OpenvinoDetector::from_config(
    EnemyColor color,
    const std::string& config_file)
{
    auto param = static_param::parse_file(config_file);

    OpenvinoConfig config;
    config.model_path = std::string(ASSET_DIR) + "/" + static_param::get_param<std::string>(
        param, "Detector.yolo", "model");
    config.input_size = static_cast<int>(static_param::get_param<int64_t>(
        param, "Detector.yolo", "input_size"));
    config.conf_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector.yolo", "conf_threshold"));
    config.nms_threshold = static_cast<float>(static_param::get_param<double>(
        param, "Detector.yolo", "nms_threshold"));
    config.device = static_param::get_param<std::string>(
        param, "Detector.yolo.openvino", "device");

    return std::make_unique<OpenvinoDetector>(config, color);
}

OpenvinoDetector::~OpenvinoDetector() = default;

// ============================================================================
// 检测接口
// ============================================================================

std::vector<DetectedArmor> OpenvinoDetector::detect(const cv::Mat& image)
{
    // 预处理
    auto [resized, scale, dx, dy] = preprocess(image);

    // 创建输入张量
    ov::Tensor input_tensor = ov::Tensor(
        compiled_model_.input().get_element_type(),
        compiled_model_.input().get_shape(),
        resized.data
    );

    // 推理
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.infer();

    // 后处理
    const ov::Tensor& output = infer_request_.get_output_tensor();
    auto detections = postprocess(output, scale, dx, dy);

    // 保存调试信息 (仅在需要时)
    last_detections_ = detections;

    return detections;
}

// ============================================================================
// 异步接口实现 (参考 sp_vision_25)
// ============================================================================

void OpenvinoDetector::push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
                            const serial::SerialReceiveData& serial_data)
{
    // 记录提交时间 (用于计算端到端延迟)
    auto submit_time = std::chrono::steady_clock::now();

    // 队列过长时直接丢弃新帧 (不阻塞)
    {
        std::lock_guard lock(task_mutex_);
        constexpr size_t MAX_QUEUE_SIZE = 2;  // 低延迟模式：最多2帧
        if (task_queue_.size() >= MAX_QUEUE_SIZE) {
            return;  // 丢弃当前帧
        }
    }

    // 预处理
    auto [resized, scale, dx, dy] = preprocess(image);

    // 创建新的 InferRequest (每帧一个，支持并行)
    auto infer_request = compiled_model_.create_infer_request();

    // 创建输入张量
    ov::Tensor input_tensor = ov::Tensor(
        compiled_model_.input().get_element_type(),
        compiled_model_.input().get_shape(),
        resized.data
    );

    // 设置输入并异步启动推理
    infer_request.set_input_tensor(input_tensor);
    infer_request.start_async();

    // 入队
    {
        std::lock_guard lock(task_mutex_);
        task_queue_.push(InferenceTask{
            std::move(infer_request),
            image.clone(),  // 保存原始图像用于可视化
            scale, dx, dy,
            frame_id, timestamp_us,
            serial_data,
            submit_time  // 使用 push 开始时记录的时间
        });
    }
    task_cv_.notify_one();
}

AsyncDetectionResult OpenvinoDetector::pop()
{
    InferenceTask task;

    // 出队
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

    // 后处理
    const ov::Tensor& output = task.request.get_output_tensor();
    auto armors = postprocess(output, task.scale, task.dx, task.dy);

    return AsyncDetectionResult{
        std::move(armors),
        std::move(task.image),
        task.frame_id,
        task.timestamp_us,
        task.serial_data,
        latency_ms
    };
}

size_t OpenvinoDetector::queue_size() const {
    std::lock_guard lock(task_mutex_);
    return task_queue_.size();
}

// ============================================================================
// 内部方法
// ============================================================================

std::tuple<cv::Mat, float, int, int> OpenvinoDetector::preprocess(const cv::Mat& image)
{
    int target_size = config_.input_size;

    // Letterbox resize (保持宽高比)
    float scale = std::min(
        static_cast<float>(target_size) / image.cols,
        static_cast<float>(target_size) / image.rows
    );

    int new_w = static_cast<int>(image.cols * scale);
    int new_h = static_cast<int>(image.rows * scale);

    cv::Mat resized;
    // INTER_NEAREST 最快，检测任务对图像质量不敏感
    cv::resize(image, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_NEAREST);

    // 创建灰色背景
    cv::Mat padded(target_size, target_size, CV_8UC3, cv::Scalar(114, 114, 114));

    // 居中放置
    int dx = (target_size - new_w) / 2;
    int dy = (target_size - new_h) / 2;
    resized.copyTo(padded(cv::Rect(dx, dy, new_w, new_h)));

    return {padded, scale, dx, dy};
}

std::vector<DetectedArmor> OpenvinoDetector::postprocess(
    const ov::Tensor& output,
    float scale,
    int dx,
    int dy)
{
    ov::Shape shape = output.get_shape();
    // shape: [1, num_detections, 22]
    int num_detections = static_cast<int>(shape[1]);
    int num_features = static_cast<int>(shape[2]);

    cv::Mat output_mat(num_detections, num_features, CV_32F,
                       const_cast<float*>(output.data<float>()));

    std::vector<DetectedArmor> candidates;
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;

    // detect_color_ 是串口传来的 enemy_color (敌方颜色)
    // enemy_color=1(敌方红) → 检测红色装甲板
    // enemy_color=2(敌方蓝) → 检测蓝色装甲板
    int target_color_idx = (detect_color_ == EnemyColor::RED)
                           ? model_color::RED
                           : model_color::BLUE;

    for (int i = 0; i < num_detections; ++i) {
        // 置信度过滤
        float conf = sigmoid(output_mat.at<float>(i, output_idx::CONFIDENCE));
        if (conf < config_.conf_threshold) {
            continue;
        }

        // 解析颜色
        cv::Mat color_scores = output_mat.row(i).colRange(
            output_idx::COLOR_START, output_idx::COLOR_END);
        cv::Point color_idx;
        double color_score;
        cv::minMaxLoc(color_scores, nullptr, &color_score, nullptr, &color_idx);
        int color = color_idx.x;

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
        cv::Mat class_scores = output_mat.row(i).colRange(
            output_idx::CLASS_START, output_idx::CLASS_END);
        cv::Point class_idx;
        double class_score;
        cv::minMaxLoc(class_scores, nullptr, &class_score, nullptr, &class_idx);
        int label = class_idx.x;

        // 解析关键点 (模型输出是左上角逆时针，与我们的约定一致)
        // 需要减去 letterbox 偏移再除以缩放比例
        std::vector<cv::Point2f> landmarks(4);
        for (int j = 0; j < 4; ++j) {
            float x = output_mat.at<float>(i, output_idx::LANDMARKS_START + j * 2);
            float y = output_mat.at<float>(i, output_idx::LANDMARKS_START + j * 2 + 1);
            // 还原到原图坐标: (x - dx) / scale
            landmarks[j] = cv::Point2f((x - dx) / scale, (y - dy) / scale);
        }

        // 计算包围盒 (用于 NMS)
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

        // 计算长宽比 (用于判断大小装甲板)
        float length = cv::norm(landmarks[0] - landmarks[3]);  // 左上到右上
        float width = cv::norm(landmarks[0] - landmarks[1]);   // 左上到左下
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

    return results;
}

float OpenvinoDetector::sigmoid(float x)
{
    if (x > 0) {
        return 1.0f / (1.0f + std::exp(-x));
    } else {
        float exp_x = std::exp(x);
        return exp_x / (1.0f + exp_x);
    }
}

ArmorNumber OpenvinoDetector::label_to_armor_number(int label)
{
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

ArmorType OpenvinoDetector::get_armor_type(int label, float ratio)
{
    // 英雄和基地大装甲板是大装甲板
    if (label == model_class::HERO || label == model_class::BASE_B) {
        return ArmorType::LARGE;
    }

    // 基地小是小装甲板
    if (label == model_class::BASE_S) {
        return ArmorType::SMALL;
    }

    // 其他根据长宽比判断
    // 大装甲板长宽比约 4.5:1，小装甲板约 2.6:1
    // 阈值取 3.5
    return (ratio > 3.5f) ? ArmorType::LARGE : ArmorType::SMALL;
}

}  // namespace autoaim::detector
