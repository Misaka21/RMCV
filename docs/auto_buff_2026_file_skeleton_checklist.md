# AutoBuff 2026 逐文件 `.hpp/.cpp` 空实现骨架清单

> 用途：给实现者按文件逐个创建“可编译空骨架”，再逐步填充逻辑。
> 
> 约定：
> 1. 与当前项目风格保持一致（include guard + namespace + logger/runtime_param 风格）。
> 2. 先保证链接通过，再逐步填算法。
> 3. 所有 runtime 参数在使用点直接 `runtime_param::get_param<T>()`，不缓存封装。

---

## 0. 执行顺序（强制）

1. 先建类型头文件（`common/types.hpp`, `observation/types.hpp`, `predictor/types.hpp`, `fire_control/types.hpp`）。
2. 再建 detector core（`raw_types/preprocess/postprocess/decoder`）。
3. 再建 OV/TRT backend 类骨架（先返回空结果但接口完整）。
4. 再建 observation 层（indexer/builder/debouncer/instance_manager）。
5. 再建 predictor models（const/small_ekf/large_lsm/mode_manager）。
6. 最后建 fire_control（ranker/coop/controller）。
7. 最后改 CMake，确保目标可编译。

---

## 1. 文件树与创建清单

```text
aimer/auto_buff/
  common/
    types.hpp                            # 修改

  detector/
    common/
      raw_types.hpp                      # 新增
      preprocess.hpp                     # 新增
      preprocess.cpp                     # 新增
      postprocess.hpp                    # 新增
      postprocess.cpp                    # 新增
    decoder/
      buff_decoder.hpp                   # 新增
      sp25_decoder.hpp                   # 新增
      sp25_decoder.cpp                   # 新增
    detector_ov/
      openvino_detector.hpp              # 新增
      openvino_detector.cpp              # 新增
      CMakeLists.txt                     # 新增
    detector_trt/
      tensorrt_detector.hpp              # 新增
      tensorrt_detector.cpp              # 新增
      cuda_preprocess.hpp                # 新增(可占位)
      cuda_preprocess.cu                 # 新增(可占位)
      int8_calibrator.hpp                # 新增(可占位)
      FindTensorRT.cmake                 # 新增(可占位)
      CMakeLists.txt                     # 新增
    detector_factory.cpp                 # 修改
    detector_factory.hpp                 # 修改
    CMakeLists.txt                       # 修改

  observation/
    types.hpp                            # 新增
    slot_indexer.hpp                     # 新增
    slot_indexer.cpp                     # 新增
    observation_builder.hpp              # 新增
    observation_builder.cpp              # 新增
    slot_debouncer.hpp                   # 新增
    slot_debouncer.cpp                   # 新增
    instance_manager.hpp                 # 新增
    instance_manager.cpp                 # 新增
    CMakeLists.txt                       # 新增

  predictor/
    mode_manager.hpp                     # 新增
    mode_manager.cpp                     # 新增
    models/
      model_interface.hpp                # 新增
      const_model.hpp                    # 新增
      const_model.cpp                    # 新增
      small_ekf_model.hpp                # 新增
      small_ekf_model.cpp                # 新增
      large_lsm_model.hpp                # 新增
      large_lsm_model.cpp                # 新增
      CMakeLists.txt                     # 新增
    buff_predictor.hpp                   # 修改
    buff_predictor.cpp                   # 修改
    predictor_node.cpp                   # 修改
    predictor_node.hpp                   # 可保持
    CMakeLists.txt                       # 修改

  fire_control/
    types.hpp                            # 新增
    target_ranker.hpp                    # 新增
    target_ranker.cpp                    # 新增
    coop_policy.hpp                      # 新增
    coop_policy.cpp                      # 新增
    fire_controller.hpp                  # 修改
    fire_controller.cpp                  # 修改
    fire_control_node.cpp                # 修改
    CMakeLists.txt                       # 修改
```

---

## 2. 骨架模板（按文件）

## 2.1 `aimer/auto_buff/detector/common/raw_types.hpp`

```cpp
#ifndef AIMER_AUTOBUFF_DETECTOR_COMMON_RAW_TYPES_HPP
#define AIMER_AUTOBUFF_DETECTOR_COMMON_RAW_TYPES_HPP

#include <array>
#include <cstdint>

#include <opencv2/core.hpp>

namespace autobuff::detector {

struct LetterboxMeta {
    int src_w = 0;
    int src_h = 0;
    int net_w = 640;
    int net_h = 640;
    float scale = 1.f;
    float pad_x = 0.f;
    float pad_y = 0.f;
};

enum class RawClassId : int {
    FAN = 0,
    R = 1,
    UNKNOWN = -1,
};

struct RawBuffObject {
    cv::Rect2f box{};
    RawClassId cls = RawClassId::UNKNOWN;
    float score = 0.f;

    std::array<cv::Point2f, 8> kpts{};
    uint8_t kpt_count = 0;

    float fan_score = 0.f;
    float r_score = 0.f;
    float lit_score = 0.f;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_COMMON_RAW_TYPES_HPP
```

Checklist:
- [ ] include guard 命名与路径一致
- [ ] `LetterboxMeta`、`RawClassId`、`RawBuffObject` 字段齐全

---

## 2.2 `aimer/auto_buff/detector/common/preprocess.hpp`

```cpp
#ifndef AIMER_AUTOBUFF_DETECTOR_COMMON_PREPROCESS_HPP
#define AIMER_AUTOBUFF_DETECTOR_COMMON_PREPROCESS_HPP

#include <opencv2/core.hpp>

#include "raw_types.hpp"

namespace autobuff::detector {

cv::Mat letterbox_bgr_u8(
    const cv::Mat& src,
    int net_size,
    LetterboxMeta& meta);

void bgr_to_chw_f32(
    const cv::Mat& bgr,
    float* out,
    bool normalize,
    bool bgr2rgb);

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_COMMON_PREPROCESS_HPP
```

## 2.3 `aimer/auto_buff/detector/common/preprocess.cpp`

```cpp
#include "preprocess.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace autobuff::detector {

cv::Mat letterbox_bgr_u8(const cv::Mat& src, int net_size, LetterboxMeta& meta) {
    // TODO: 实现标准 letterbox
    meta.src_w = src.cols;
    meta.src_h = src.rows;
    meta.net_w = net_size;
    meta.net_h = net_size;
    meta.scale = 1.f;
    meta.pad_x = 0.f;
    meta.pad_y = 0.f;
    if (src.empty()) {
        return cv::Mat();
    }
    cv::Mat out;
    cv::resize(src, out, cv::Size(net_size, net_size));
    return out;
}

void bgr_to_chw_f32(const cv::Mat& bgr, float* out, bool normalize, bool bgr2rgb) {
    // TODO: 实现 HWC->CHW
    (void)bgr;
    (void)out;
    (void)normalize;
    (void)bgr2rgb;
}

}  // namespace autobuff::detector
```

Checklist:
- [ ] 空实现可编译
- [ ] 先用简单 resize 占位，后续替换 true letterbox

---

## 2.4 `aimer/auto_buff/detector/common/postprocess.hpp`

```cpp
#ifndef AIMER_AUTOBUFF_DETECTOR_COMMON_POSTPROCESS_HPP
#define AIMER_AUTOBUFF_DETECTOR_COMMON_POSTPROCESS_HPP

#include <vector>

#include <opencv2/core.hpp>

#include "aimer/auto_buff/common/types.hpp"
#include "aimer/auto_buff/detector/common/raw_types.hpp"
#include "aimer/common/robot_state.hpp"

namespace autobuff::detector {

struct PostprocessConfig {
    float lit_thresh = 0.50f;
    float min_target_conf = 0.35f;
    float min_r_conf = 0.35f;
    double slot_phase_bias = 0.0;
};

class Postprocessor {
public:
    explicit Postprocessor(PostprocessConfig cfg);

    autobuff::BuffDetectionResult build_result(
        const std::vector<RawBuffObject>& objs,
        const cv::Mat& image,
        double timestamp,
        int frame_id,
        const aimer::RobotState& state,
        float latency_ms,
        autobuff::EnemyColor enemy_color,
        autobuff::DetectorBackend backend) const;

private:
    PostprocessConfig cfg_;

    cv::Point2f choose_r_center(const std::vector<RawBuffObject>& objs, bool& valid, float& conf) const;
    double calc_angle(const cv::Point2f& p, const cv::Point2f& c) const;
    int angle_to_slot(double angle) const;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_COMMON_POSTPROCESS_HPP
```

## 2.5 `aimer/auto_buff/detector/common/postprocess.cpp`

```cpp
#include "postprocess.hpp"

#include <cmath>

namespace autobuff::detector {

Postprocessor::Postprocessor(PostprocessConfig cfg) : cfg_(cfg) {}

autobuff::BuffDetectionResult Postprocessor::build_result(
    const std::vector<RawBuffObject>& objs,
    const cv::Mat& image,
    double timestamp,
    int frame_id,
    const aimer::RobotState& state,
    float latency_ms,
    autobuff::EnemyColor enemy_color,
    autobuff::DetectorBackend backend) const {

    autobuff::BuffDetectionResult out;
    out.backend = backend;
    out.timestamp = timestamp;
    out.frame_id = frame_id;
    out.robot_state = state;
    out.latency_ms = latency_ms;
    out.enemy_color = enemy_color;
    out.image = image;

    // TODO: r_center 选择 + fan 分配 + slot 冲突处理
    // TODO: out.targets 填充

    out.recompute_summary();
    return out;
}

cv::Point2f Postprocessor::choose_r_center(const std::vector<RawBuffObject>& objs, bool& valid, float& conf) const {
    (void)objs;
    valid = false;
    conf = 0.f;
    return {};
}

double Postprocessor::calc_angle(const cv::Point2f& p, const cv::Point2f& c) const {
    return std::atan2(-(p.y - c.y), p.x - c.x);
}

int Postprocessor::angle_to_slot(double angle) const {
    const double step = 2.0 * M_PI / autobuff::NUM_SLOTS;
    double x = angle - cfg_.slot_phase_bias;
    while (x <= -M_PI) x += 2.0 * M_PI;
    while (x > M_PI) x -= 2.0 * M_PI;
    int s = static_cast<int>(std::llround(x / step));
    s %= autobuff::NUM_SLOTS;
    if (s < 0) s += autobuff::NUM_SLOTS;
    return s;
}

}  // namespace autobuff::detector
```

Checklist:
- [ ] 保证 `build_result` 即使空输入也返回合法对象
- [ ] `recompute_summary()` 必须调用

---

## 2.6 `aimer/auto_buff/detector/decoder/buff_decoder.hpp`

```cpp
#ifndef AIMER_AUTOBUFF_DETECTOR_DECODER_BUFF_DECODER_HPP
#define AIMER_AUTOBUFF_DETECTOR_DECODER_BUFF_DECODER_HPP

#include <vector>

#include "aimer/auto_buff/detector/common/raw_types.hpp"

namespace autobuff::detector {

class IBuffDecoder {
public:
    virtual ~IBuffDecoder() = default;

    virtual std::vector<RawBuffObject> decode(
        const float* out,
        const std::vector<int64_t>& shape,
        const LetterboxMeta& meta) const = 0;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_DECODER_BUFF_DECODER_HPP
```

---

## 2.7 `aimer/auto_buff/detector/decoder/sp25_decoder.hpp`

```cpp
#ifndef AIMER_AUTOBUFF_DETECTOR_DECODER_SP25_DECODER_HPP
#define AIMER_AUTOBUFF_DETECTOR_DECODER_SP25_DECODER_HPP

#include "buff_decoder.hpp"

namespace autobuff::detector {

class Sp25Decoder final : public IBuffDecoder {
public:
    struct Config {
        float conf_thres = 0.45f;
        float nms_thres = 0.45f;
        int max_det = 64;

        int fan_class_id = 0;
        int r_class_id = 1;
        int kpt_center_idx = 4;
        int kpt_r_hint_idx = 5;
    };

    explicit Sp25Decoder(Config cfg);

    std::vector<RawBuffObject> decode(
        const float* out,
        const std::vector<int64_t>& shape,
        const LetterboxMeta& meta) const override;

private:
    Config cfg_;

    static bool parse_layout(const std::vector<int64_t>& shape, int& c, int& n, bool& chw_like);
    RawBuffObject decode_one(const float* ptr, int c, const LetterboxMeta& meta) const;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_DECODER_SP25_DECODER_HPP
```

## 2.8 `aimer/auto_buff/detector/decoder/sp25_decoder.cpp`

```cpp
#include "sp25_decoder.hpp"

#include <algorithm>

namespace autobuff::detector {

Sp25Decoder::Sp25Decoder(Config cfg) : cfg_(cfg) {}

bool Sp25Decoder::parse_layout(const std::vector<int64_t>& shape, int& c, int& n, bool& chw_like) {
    // TODO: 支持 [1,C,N] 和 [1,N,C]
    c = 0;
    n = 0;
    chw_like = true;
    if (shape.size() != 3) {
        return false;
    }
    // 占位逻辑
    c = static_cast<int>(shape[1]);
    n = static_cast<int>(shape[2]);
    return true;
}

RawBuffObject Sp25Decoder::decode_one(const float* ptr, int c, const LetterboxMeta& meta) const {
    (void)c;
    (void)meta;
    RawBuffObject o;
    if (ptr == nullptr) {
        return o;
    }
    // TODO: 解析 bbox + score + keypoints
    return o;
}

std::vector<RawBuffObject> Sp25Decoder::decode(
    const float* out,
    const std::vector<int64_t>& shape,
    const LetterboxMeta& meta) const {

    std::vector<RawBuffObject> objects;
    if (out == nullptr) {
        return objects;
    }

    int c = 0;
    int n = 0;
    bool chw_like = true;
    if (!parse_layout(shape, c, n, chw_like)) {
        return objects;
    }

    // TODO: 遍历候选 + conf 过滤 + NMS
    (void)meta;
    (void)chw_like;
    (void)c;
    (void)n;

    return objects;
}

}  // namespace autobuff::detector
```

Checklist:
- [ ] 可编译
- [ ] 先空实现返回空 vector，后续逐步填

---

## 2.9 `aimer/auto_buff/detector/detector_ov/openvino_detector.hpp`

```cpp
#ifndef AIMER_AUTOBUFF_DETECTOR_OV_OPENVINO_DETECTOR_HPP
#define AIMER_AUTOBUFF_DETECTOR_OV_OPENVINO_DETECTOR_HPP

#include <atomic>
#include <memory>
#include <thread>

#include <openvino/openvino.hpp>

#include "aimer/auto_buff/detector/common/detector_interface.hpp"
#include "aimer/auto_buff/detector/common/postprocess.hpp"
#include "aimer/auto_buff/detector/decoder/buff_decoder.hpp"

namespace autobuff::detector {

class OpenvinoBuffDetector final : public BuffDetectorInterface {
public:
    static std::unique_ptr<OpenvinoBuffDetector> from_config(
        EnemyColor color,
        const std::string& config_file = "buff.toml");

    OpenvinoBuffDetector(EnemyColor color,
                         std::string model_path,
                         int input_size,
                         std::unique_ptr<IBuffDecoder> decoder,
                         PostprocessConfig post_cfg);

    ~OpenvinoBuffDetector() override;

    BuffDetectionResult detect(const cv::Mat& image, double timestamp) override;

    void set_enemy_color(EnemyColor color) override;
    EnemyColor get_enemy_color() const override;

    bool is_async() const override { return true; }

    void push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
              const serial::SerialReceiveData& serial_data) override;
    AsyncBuffDetectionResult pop() override;

private:
    BuffDetectionResult run_once(
        const cv::Mat& image,
        double timestamp,
        int frame_id,
        const serial::SerialReceiveData* serial_data,
        float& latency_ms);

    EnemyColor color_ = EnemyColor::UNKNOWN;

    ov::Core core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_;
    ov::InferRequest infer_;

    int input_size_ = 640;

    std::unique_ptr<IBuffDecoder> decoder_;
    Postprocessor post_;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_OV_OPENVINO_DETECTOR_HPP
```

## 2.10 `aimer/auto_buff/detector/detector_ov/openvino_detector.cpp`

```cpp
#include "openvino_detector.hpp"

#include <chrono>

#include "aimer/auto_buff/detector/common/preprocess.hpp"
#include "aimer/auto_buff/detector/decoder/sp25_decoder.hpp"
#include "plugin/param/static_config.hpp"

namespace autobuff::detector {

std::unique_ptr<OpenvinoBuffDetector> OpenvinoBuffDetector::from_config(
    EnemyColor color,
    const std::string& config_file) {
    // TODO: 读取参数 + 创建 decoder + 返回实例
    (void)config_file;
    auto decoder = std::make_unique<Sp25Decoder>(Sp25Decoder::Config{});
    return std::make_unique<OpenvinoBuffDetector>(
        color, "", 640, std::move(decoder), PostprocessConfig{});
}

OpenvinoBuffDetector::OpenvinoBuffDetector(EnemyColor color,
                                           std::string model_path,
                                           int input_size,
                                           std::unique_ptr<IBuffDecoder> decoder,
                                           PostprocessConfig post_cfg)
    : color_(color),
      input_size_(input_size),
      decoder_(std::move(decoder)),
      post_(post_cfg) {
    (void)model_path;
    // TODO: ov core read/compile model
}

OpenvinoBuffDetector::~OpenvinoBuffDetector() = default;

void OpenvinoBuffDetector::set_enemy_color(EnemyColor color) { color_ = color; }
EnemyColor OpenvinoBuffDetector::get_enemy_color() const { return color_; }

BuffDetectionResult OpenvinoBuffDetector::run_once(
    const cv::Mat& image,
    double timestamp,
    int frame_id,
    const serial::SerialReceiveData* serial_data,
    float& latency_ms) {

    auto t0 = std::chrono::steady_clock::now();

    // TODO: preprocess + infer + decode
    std::vector<RawBuffObject> objs;

    aimer::RobotState rs;
    if (serial_data != nullptr) {
        // TODO: build robot state if needed
    }

    auto t1 = std::chrono::steady_clock::now();
    latency_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    return post_.build_result(
        objs, image, timestamp, frame_id, rs, latency_ms, color_, autobuff::DetectorBackend::OPENVINO);
}

BuffDetectionResult OpenvinoBuffDetector::detect(const cv::Mat& image, double timestamp) {
    float latency = 0.f;
    return run_once(image, timestamp, 0, nullptr, latency);
}

void OpenvinoBuffDetector::push(const cv::Mat& image, int frame_id, int64_t timestamp_us,
                                const serial::SerialReceiveData& serial_data) {
    float latency = 0.f;
    double ts = timestamp_us / 1e6;
    BuffDetectionResult det = run_once(image, ts, frame_id, &serial_data, latency);

    AsyncBuffDetectionResult out;
    out.detection = std::move(det);
    out.image = image;
    out.frame_id = frame_id;
    out.timestamp_us = timestamp_us;
    out.serial_data = serial_data;
    out.latency_ms = latency;

    {
        std::lock_guard lock(queue_mutex_);
        result_queue_.push(std::move(out));
    }
    queue_cv_.notify_one();
}

AsyncBuffDetectionResult OpenvinoBuffDetector::pop() {
    return BuffDetectorInterface::pop();
}

}  // namespace autobuff::detector
```

Checklist:
- [ ] from_config 读取 `Detector.yolo.*`
- [ ] run_once 中 `RobotState` 从 `serial_data` 正确构建

---

## 2.11 `aimer/auto_buff/detector/detector_trt/tensorrt_detector.hpp`

> 与 OpenVINO 头文件同形，类名改为 `TensorrtBuffDetector`。

## 2.12 `aimer/auto_buff/detector/detector_trt/tensorrt_detector.cpp`

> 与 OV `.cpp` 同流程，推理后端替换 TRT；先留 TODO。

Checklist:
- [ ] 接口与 `OpenvinoBuffDetector` 对齐（便于工厂切换）
- [ ] `is_async()==true`

---

## 2.13 `aimer/auto_buff/detector/detector_factory.hpp`

```cpp
// 新增 backend 说明：
// [Detector]
// type = "yolo"
// [Detector.yolo]
// backend = "openvino" | "tensorrt"
```

Checklist:
- [ ] 注释更新
- [ ] 接口签名保持不变

## 2.14 `aimer/auto_buff/detector/detector_factory.cpp`

骨架要点：
1. `traditional` 走原逻辑。
2. `yolo` 读取 backend：
   - `openvino` -> `OpenvinoBuffDetector::from_config`
   - `tensorrt` -> `TensorrtBuffDetector::from_config`
3. 后端未编译时抛明确异常（与 auto_aim 一致）。

Checklist:
- [ ] `#ifdef ENABLE_OPENVINO_DETECTOR`
- [ ] `#ifdef ENABLE_TENSORRT_DETECTOR`
- [ ] 异常提示含“安装并重新编译”

---

## 2.15 `aimer/auto_buff/observation/types.hpp`

> 直接按主规格中的类型定义拷贝创建。

Checklist:
- [ ] `DebounceState`、`SlotMeasurement`、`BuffFrameObservation`
- [ ] `SlotTrackState`、`StableSlotObservation`、`DebouncedBuffObservation`

---

## 2.16 `aimer/auto_buff/observation/slot_indexer.hpp/.cpp`

```cpp
// hpp
class SlotIndexer {
public:
    struct Config { double phase_bias = 0.0; };
    explicit SlotIndexer(Config cfg = {});
    int angle_to_slot(double angle) const;
    double slot_to_angle(int slot_id) const;
private:
    Config cfg_;
};
```

```cpp
// cpp
int SlotIndexer::angle_to_slot(double angle) const {
    // TODO: 实现 72° 量化
}

double SlotIndexer::slot_to_angle(int slot_id) const {
    // TODO: 反映射
}
```

Checklist:
- [ ] 跨 `-pi/pi` 边界处理正确

---

## 2.17 `aimer/auto_buff/observation/observation_builder.hpp/.cpp`

```cpp
// hpp
class ObservationBuilder {
public:
    BuffFrameObservation build(const autobuff::BuffDetectionResult& det) const;
private:
    bool solve_group_pose(const autobuff::BuffDetectionResult& det,
                          Eigen::Vector3d& center_cam,
                          Eigen::Vector3d& normal_cam,
                          std::array<Eigen::Vector3d, autobuff::NUM_SLOTS>& slot_cam) const;
};
```

```cpp
// cpp
BuffFrameObservation ObservationBuilder::build(const autobuff::BuffDetectionResult& det) const {
    BuffFrameObservation out;
    // TODO: 映射 slots + PnP 填 pose
    return out;
}
```

Checklist:
- [ ] PnP 最少点数保护
- [ ] 失败时 `pose_valid=false` 但不崩溃

---

## 2.18 `aimer/auto_buff/observation/slot_debouncer.hpp/.cpp`

```cpp
// hpp
class SlotDebouncer {
public:
    struct Config {
        int on_frames = 3;
        int off_frames = 4;
        double missing_timeout = 0.12;
    };
    explicit SlotDebouncer(Config cfg = {});
    void reset();
    DebouncedBuffObservation update(const BuffFrameObservation& in);
private:
    Config cfg_;
    std::array<SlotTrackState, autobuff::NUM_SLOTS> tracks_{};
};
```

```cpp
// cpp
DebouncedBuffObservation SlotDebouncer::update(const BuffFrameObservation& in) {
    DebouncedBuffObservation out;
    // TODO: 状态机
    return out;
}
```

Checklist:
- [ ] `OFF->ON` 置 `state_changed=true`
- [ ] `missing_timeout` 强制 OFF

---

## 2.19 `aimer/auto_buff/observation/instance_manager.hpp/.cpp`

```cpp
class InstanceManager {
public:
    void reset();
    void stamp(DebouncedBuffObservation& obs);
private:
    std::array<uint32_t, autobuff::NUM_SLOTS> counters_{};
    std::array<bool, autobuff::NUM_SLOTS> last_lit_{};
};
```

Checklist:
- [ ] 只在 `false->true` 时 `counter++`

---

## 2.20 `aimer/auto_buff/predictor/mode_manager.hpp/.cpp`

```cpp
class ModeManager {
public:
    struct Config {
        int enter_large_active_frames = 3;
        int exit_large_active_frames = 4;
    };
    explicit ModeManager(Config cfg = {});
    void reset();
    autobuff::BuffMode update(const observation::DebouncedBuffObservation& obs);
    autobuff::BuffMode current() const;
private:
    Config cfg_;
    autobuff::BuffMode mode_ = autobuff::BuffMode::UNKNOWN;
    int active_streak_ = 0;
    int inactive_streak_ = 0;
};
```

Checklist:
- [ ] mode 切换点打印日志

---

## 2.21 `aimer/auto_buff/predictor/models/model_interface.hpp`

```cpp
class MotionModelInterface {
public:
    virtual ~MotionModelInterface() = default;
    virtual void reset() = 0;
    virtual void feed(const observation::DebouncedBuffObservation& obs, int track_slot) = 0;
    virtual MotionEstimate estimate() const = 0;
    virtual double delta_theta(double t_abs, double dt) const = 0;
};
```

---

## 2.22 `aimer/auto_buff/predictor/models/const_model.hpp/.cpp`

Checklist:
- [ ] 字段：`dir_`, `omega_`, `dir_votes_`, `last_phi_`, `last_t_`
- [ ] `estimate()` 返回 `SpeedModel::CONST_OMEGA`

---

## 2.23 `aimer/auto_buff/predictor/models/small_ekf_model.hpp/.cpp`

Checklist:
- [ ] 使用 `AdaptiveEkf<2,1>`
- [ ] `feed` 中做角度 unwrap
- [ ] `dt` 异常（过小/过大）时重置

---

## 2.24 `aimer/auto_buff/predictor/models/large_lsm_model.hpp/.cpp`

Checklist:
- [ ] 维护 `samples_` 滑窗
- [ ] Ceres cost 函数用 Huber
- [ ] 参数边界固定
- [ ] `residual_rms` 评估并设置 valid

---

## 2.25 `aimer/auto_buff/predictor/buff_predictor.hpp`

```cpp
class BuffPredictor {
public:
    BuffPredictor();
    void reset();
    BuffSnapshot predict(const autobuff::BuffDetectionResult& det);
private:
    observation::ObservationBuilder obs_builder_;
    observation::SlotDebouncer debouncer_;
    observation::InstanceManager instance_mgr_;

    ModeManager mode_mgr_;

    models::ConstModel const_model_;
    models::SmallEkfModel small_model_;
    models::LargeLsmModel large_model_;

    int last_track_slot_ = -1;

    int choose_track_slot(const observation::DebouncedBuffObservation& obs) const;
    int choose_recommended_slot(const BuffSnapshot& snap) const;
    void build_ccw_rank(BuffSnapshot& snap) const;
};
```

## 2.26 `aimer/auto_buff/predictor/buff_predictor.cpp`

Checklist:
- [ ] 流程顺序固定：build -> debounce -> stamp -> mode -> model -> snapshot
- [ ] `predict_timestamp` 不在这里写，由 node 写
- [ ] snapshot 无效时字段清零并返回

---

## 2.27 `aimer/auto_buff/fire_control/types.hpp`

> 按主规格类型定义创建。

---

## 2.28 `aimer/auto_buff/fire_control/target_ranker.hpp/.cpp`

```cpp
class TargetRanker {
public:
    explicit TargetRanker(const FireControlConfig& cfg);
    std::vector<SlotAimCandidate> build(
        const predictor::BuffSnapshot& snap,
        const ::fire_control::LatencyInfo& latency,
        const ::fire_control::GimbalState& gimbal) const;
private:
    FireControlConfig cfg_;
};
```

Checklist:
- [ ] 同时填 `tracking_error` 和 `score`
- [ ] 弹道无解候选保留但 `ballistic_valid=false`

---

## 2.29 `aimer/auto_buff/fire_control/coop_policy.hpp/.cpp`

```cpp
class CoopPolicy {
public:
    explicit CoopPolicy(const FireControlConfig& cfg);
    int select(const predictor::BuffSnapshot& snap,
               const std::vector<SlotAimCandidate>& cands,
               bool& coop_applied) const;
private:
    FireControlConfig cfg_;
};
```

Checklist:
- [ ] FIRST/SECOND 逻辑与回退规则完整

---

## 2.30 `aimer/auto_buff/fire_control/fire_controller.hpp/.cpp`

Checklist:
- [ ] `control` 更新 `gimbal_state_`
- [ ] 调 `ranker_.build` + `coop_.select`
- [ ] `allow_refire` 与 `mark_fired` 完整
- [ ] no target 输出 `control_enabled=false`

---

## 2.31 `aimer/auto_buff/fire_control/fire_control_node.cpp`

Checklist:
- [ ] 保持 500Hz
- [ ] `predict_to_send` 更新逻辑不变
- [ ] 模式切换 reset controller

---

## 2.32 `aimer/auto_buff/detector/CMakeLists.txt`

Checklist:
- [ ] `find_package(OpenVINO QUIET)`
- [ ] `find_package(TensorRT QUIET)`
- [ ] option: `ENABLE_OPENVINO_DETECTOR`, `ENABLE_TENSORRT_DETECTOR`
- [ ] 按宏链接后端

---

## 2.33 `aimer/auto_buff/observation/CMakeLists.txt`

骨架：
```cmake
add_library(buff_observation STATIC
    slot_indexer.cpp
    observation_builder.cpp
    slot_debouncer.cpp
    instance_manager.cpp
)

target_include_directories(buff_observation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(buff_observation PUBLIC
    auto_buff_common
    aimer_common
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
)
```

---

## 2.34 `aimer/auto_buff/predictor/CMakeLists.txt`

Checklist:
- [ ] 链接 `buff_observation`
- [ ] 链接 `ceres`
- [ ] 新增 `models/*.cpp`

---

## 2.35 `aimer/auto_buff/fire_control/CMakeLists.txt`

Checklist:
- [ ] 新增 `target_ranker.cpp`、`coop_policy.cpp`
- [ ] 链接 `buff_predictor` 与 `aimer_common`

---

## 3. 开工时的最小编译策略

1. 第一阶段所有新类函数先返回空结果，但保证签名稳定。
2. 第二阶段只填数据贯通，不上复杂算法。
3. 第三阶段再替换算法实现（EKF/LSM/协同策略）。

---

## 4. 逐阶段 “完成定义”

## 阶段 A（骨架可编译）
- [ ] 所有新文件创建
- [ ] CMake 通过
- [ ] `RMCV2026` 可链接

## 阶段 B（数据贯通）
- [ ] `buff_detections -> buff_snapshot -> fire_command` 全链路通
- [ ] dashboard 有基础指标

## 阶段 C（算法填充）
- [ ] slot debounce 稳定
- [ ] small ekf 可输出
- [ ] large lsm 可拟合
- [ ] coop 策略生效

