# AutoBuff 2026 重构执行规格（Execution Spec）

> 目标：本文件是给实现者（Claude Code）直接落地用的执行规格。
> 
> 约束：
> 1. 保持与当前 RMCV 工程风格一致（命名、线程、UMT、日志、配置读取方式）。
> 2. 对外协议兼容（继续输出 `fire_control::FireCommand`，不强制新增上位机字段）。
> 3. 双车协同用静态配置角色（`DISABLED / CCW_FIRST / CCW_SECOND`）。
> 4. 检测后端支持 OpenVINO 与 TensorRT，选择策略与 AutoAim 一致（显式 backend，不自动回退）。

---

## 1. 规则约束到代码约束

### 1.1 比赛规则（实现必须满足）
1. 双方都可打能量机关，互不互斥。
2. 红蓝机关共轴反向旋转；每场方向随机、整场保持。
3. 小符角速度固定：`ω = π/3 rad/s`。
4. 大符非激活态：`ω = π/3 rad/s`。
5. 大符激活态速度函数：`spd(t) = a * sin(w * t) + b`，`b = 2.090 - a`。
6. 参数范围：`a ∈ [0.780, 1.045]`, `w ∈ [1.884, 2.000]`。
7. 每次进入可激活态参数重置，时间原点重置为 `t=0`。
8. 实际速度相对目标函数可存在 `<=500ms` 时间误差。
9. 小符激活时仅 1 块亮；大符激活时 2 块亮。

### 1.2 模式定义（内部统一）
```cpp
enum class BuffMode : uint8_t {
    UNKNOWN = 0,
    SMALL_ACTIVE = 1,
    LARGE_INACTIVE = 2,
    LARGE_ACTIVE = 3,
};
```

### 1.3 模式判定规则（不可留空）
1. `aim_mode == ENERGY_SMALL` -> `SMALL_ACTIVE`。
2. `aim_mode == ENERGY_LARGE` 且 `lit_count >= 2` 连续 `enter_large_active_frames` 帧 -> `LARGE_ACTIVE`。
3. `aim_mode == ENERGY_LARGE` 且 `lit_count < 2` 连续 `exit_large_active_frames` 帧 -> `LARGE_INACTIVE`。
4. 非能量机关模式 -> `UNKNOWN`，并重置预测状态。

---

## 2. 总体数据流与线程模型

## 2.1 线程拓扑（保持现有 main.cpp）
1. `autobuff::detector::background_buff_detector_run("buff.toml")`
2. `autobuff::predictor::start_predictor_node()`
3. `autobuff::fire_control::start_fire_control_node("aimer.toml")`

## 2.2 UMT 通道契约
1. Detector 输出：`Message<autobuff::BuffDetectionResult>("buff_detections")`
2. Predictor 输出：`BasicObjManager<autobuff::predictor::BuffSnapshot>("buff_snapshot")`
3. FireControl 输出：`BasicObjManager<fire_control::FireCommand>("fire_command")`

## 2.3 时间戳约束
1. 全流程使用 steady clock 秒值。
2. `BuffDetectionResult.timestamp` 必须是该图像帧时刻。
3. `BuffSnapshot.predict_timestamp` 必须是 predictor 处理完成时刻。
4. FireControl 使用 `LatencyEstimator.update_predict_to_send(now - predict_timestamp)`。

---

## 3. 目录与目标库（CMake 目标图）

```text
aimer/auto_buff/
  common/
  detector/
    common/
    decoder/
    detector_ov/
    detector_trt/
  observation/
  predictor/
    models/
  fire_control/
```

### 3.1 CMake 目标（必须创建）
1. `auto_buff_common`（已有，扩展类型）
2. `buff_detector_core`（新增：preprocess/postprocess/decoder）
3. `buff_detector_openvino`（新增，可选）
4. `buff_detector_tensorrt`（新增，可选）
5. `buff_detector_node`（改造：工厂 + node）
6. `buff_observation`（新增）
7. `buff_predictor_models`（新增）
8. `buff_predictor`（改造）
9. `buff_fire_control`（改造）

### 3.2 依赖关系
1. `buff_detector_node` -> `buff_detector_core` + (`buff_detector_openvino`/`buff_detector_tensorrt`)
2. `buff_observation` -> `auto_buff_common` + `aimer_common`
3. `buff_predictor_models` -> `buff_observation` + `ceres`
4. `buff_predictor` -> `buff_predictor_models`
5. `buff_fire_control` -> `buff_predictor` + `aimer_common`

---

## 4. 类型系统（逐头文件规范）

## 4.1 `aimer/auto_buff/common/types.hpp`

```cpp
namespace autobuff {

constexpr int NUM_SLOTS = 5;
constexpr double RUNE_RADIUS = 0.700;

enum class EnemyColor : uint8_t { UNKNOWN = 0, RED = 1, BLUE = 2 };
enum class DetectionStatus : uint8_t { NONE = 0, R_ONLY = 1, TARGETS_ONLY = 2, PARTIAL = 3, COMPLETE = 4 };
enum class BuffMode : uint8_t { UNKNOWN = 0, SMALL_ACTIVE = 1, LARGE_INACTIVE = 2, LARGE_ACTIVE = 3 };
enum class RotateDir : int8_t { UNKNOWN = 0, CW = -1, CCW = 1 };
enum class CoopRole : uint8_t { DISABLED = 0, CCW_FIRST = 1, CCW_SECOND = 2 };
enum class DetectorBackend : uint8_t { TRADITIONAL = 0, OPENVINO = 1, TENSORRT = 2 };

struct DetectedRCenter {
    cv::Point2f center{};
    std::vector<cv::Point2f> landmarks{};
    bool valid = false;
    float confidence = 0.f;
};

struct DetectedTarget {
    cv::Point2f center{};
    std::array<cv::Point2f, 8> keypoints{};
    uint8_t keypoint_count = 0;

    int slot_id = -1;
    double angle = 0.0;

    bool is_lit = false;
    bool valid = false;

    float confidence = 0.f;
    float lit_confidence = 0.f;
};

struct BuffDetectionResult {
    DetectorBackend backend = DetectorBackend::TRADITIONAL;

    DetectedRCenter r_center{};
    std::array<DetectedTarget, NUM_SLOTS> targets{};

    uint8_t lit_mask = 0;
    int target_count = 0;
    int lit_count = 0;

    DetectionStatus status = DetectionStatus::NONE;
    EnemyColor enemy_color = EnemyColor::UNKNOWN;

    int frame_id = 0;
    double timestamp = 0.0;
    float latency_ms = 0.f;

    aimer::RobotState robot_state{};
    cv::Mat image{};

    bool has_slot(int i) const;
    bool slot_lit(int i) const;
    std::vector<int> lit_slots() const;
    void recompute_summary();
};

}  // namespace autobuff
```

## 4.2 `aimer/auto_buff/observation/types.hpp`

```cpp
namespace autobuff::observation {

enum class DebounceState : uint8_t {
    UNKNOWN = 0,
    CANDIDATE_ON = 1,
    ON = 2,
    CANDIDATE_OFF = 3,
    OFF = 4,
};

struct SlotMeasurement {
    int slot_id = -1;
    bool has_measurement = false;
    bool is_lit_raw = false;

    float det_conf = 0.f;
    float lit_conf = 0.f;

    double angle = 0.0;
    cv::Point2f center_px{};

    std::array<cv::Point2f, 8> keypoints{};
    uint8_t keypoint_count = 0;
};

struct BuffFrameObservation {
    bool valid = false;
    int frame_id = 0;
    double timestamp = 0.0;

    aimer::RobotState self_state{};
    autobuff::DetectedRCenter r_center{};

    std::array<SlotMeasurement, autobuff::NUM_SLOTS> slots{};
    uint8_t raw_lit_mask = 0;
    int raw_lit_count = 0;

    autobuff::DetectionStatus status = autobuff::DetectionStatus::NONE;

    // 几何解（可选）
    bool pose_valid = false;
    Eigen::Vector3d center_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_cam = Eigen::Vector3d(0, 0, 1);
    std::array<Eigen::Vector3d, autobuff::NUM_SLOTS> pos_cam{};
};

struct SlotTrackState {
    DebounceState state = DebounceState::UNKNOWN;
    int on_count = 0;
    int off_count = 0;
    double last_update = 0.0;

    bool stable_lit = false;
    uint32_t instance_id = 0;
};

struct StableSlotObservation {
    int slot_id = -1;
    bool valid = false;
    bool is_lit = false;
    bool state_changed = false;

    uint32_t instance_id = 0;
    float confidence = 0.f;

    double angle = 0.0;
    cv::Point2f center_px{};

    Eigen::Vector3d pos_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_cam = Eigen::Vector3d::Zero();
};

struct DebouncedBuffObservation {
    bool valid = false;
    int frame_id = 0;
    double timestamp = 0.0;

    aimer::RobotState self_state{};

    Eigen::Vector3d center_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_cam = Eigen::Vector3d(0, 0, 1);

    std::array<StableSlotObservation, autobuff::NUM_SLOTS> slots{};

    uint8_t lit_mask = 0;
    int lit_count = 0;

    autobuff::RotateDir dir_hint = autobuff::RotateDir::UNKNOWN;
};

}  // namespace autobuff::observation
```

## 4.3 `aimer/auto_buff/predictor/types.hpp`

```cpp
namespace autobuff::predictor {

enum class SpeedModel : uint8_t {
    UNKNOWN = 0,
    CONST_OMEGA = 1,
    LARGE_SINE_LSM = 2,
};

struct LargeSineParam {
    bool valid = false;

    int dir = 1;               // +1 ccw, -1 cw
    double start_time = 0.0;   // t=0 absolute timestamp

    double a = 0.90;
    double w = 1.94;
    double tau = 0.0;          // [-0.5, 0.5]
    double phi0 = 0.0;

    double residual_rms = 1e9;
    int sample_count = 0;

    double b() const { return 2.090 - a; }
};

struct MotionEstimate {
    SpeedModel model = SpeedModel::UNKNOWN;
    double omega_const = 0.0;
    LargeSineParam large{};
    double confidence = 0.0;
};

struct RuneSlotState {
    bool valid = false;
    bool is_lit = false;
    uint32_t instance_id = 0;

    float confidence = 0.f;
    double angle = 0.0;
    cv::Point2f center_px{};

    Eigen::Vector3d pos_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_cam = Eigen::Vector3d::Zero();
};

struct BuffSnapshot {
    bool valid = false;

    int frame_id = 0;
    double timestamp = 0.0;
    double predict_timestamp = 0.0;

    aimer::RobotState self_state{};

    autobuff::BuffMode mode = autobuff::BuffMode::UNKNOWN;
    MotionEstimate motion{};

    Eigen::Vector3d center_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_cam = Eigen::Vector3d(0, 0, 1);

    std::array<RuneSlotState, autobuff::NUM_SLOTS> slots{};
    uint8_t lit_mask = 0;
    int lit_count = 0;

    int recommended_slot = -1;

    // ccw 排序结果
    std::array<int, autobuff::NUM_SLOTS> ccw_rank_to_slot{{-1,-1,-1,-1,-1}};
    int ranked_count = 0;

    bool has_slot(int slot) const;
    bool slot_lit(int slot) const;
    Eigen::Vector3d predict_slot_world(int slot, double dt) const;
};

}  // namespace autobuff::predictor
```

## 4.4 `aimer/auto_buff/fire_control/types.hpp`

```cpp
namespace autobuff::fire_control {

struct SlotAimCandidate {
    int slot_id = -1;
    int ccw_rank = -1;
    uint32_t instance_id = 0;

    bool is_lit = false;
    bool ballistic_valid = false;

    double impact_dt = 0.0;
    double tracking_error = 1e9;

    float confidence = 0.f;
    double score = -1e9;

    Eigen::Vector3d pred_world = Eigen::Vector3d::Zero();
    ::fire_control::AimResult aim{};
};

struct FireControlDiag {
    int chosen_slot = -1;
    int chosen_rank = -1;
    double chosen_error = 1e9;
    double chosen_score = -1e9;
    int candidate_count = 0;
    bool coop_applied = false;
};

struct FireControlConfig {
    double fire_threshold = 0.02;
    double min_confidence = 0.30;
    double max_prediction_latency = 0.35;
    double refire_block_sec = 0.08;

    double score_w_error = 1.0;
    double score_w_conf = 0.2;

    autobuff::CoopRole coop_role = autobuff::CoopRole::DISABLED;
    bool coop_only_large_active = true;
};

}  // namespace autobuff::fire_control
```

---

## 5. Detector 详细实现（OV/TRT + Decoder）

## 5.1 文件列表
1. `aimer/auto_buff/detector/common/raw_types.hpp`
2. `aimer/auto_buff/detector/common/preprocess.hpp/.cpp`
3. `aimer/auto_buff/detector/common/postprocess.hpp/.cpp`
4. `aimer/auto_buff/detector/decoder/buff_decoder.hpp`
5. `aimer/auto_buff/detector/decoder/sp25_decoder.hpp/.cpp`
6. `aimer/auto_buff/detector/detector_ov/openvino_detector.hpp/.cpp`
7. `aimer/auto_buff/detector/detector_trt/tensorrt_detector.hpp/.cpp`

## 5.2 `raw_types.hpp`
```cpp
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

    // 可选 logits
    float fan_score = 0.f;
    float r_score = 0.f;
    float lit_score = 0.f;
};

}  // namespace autobuff::detector
```

## 5.3 `buff_decoder.hpp`
```cpp
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
```

## 5.4 `sp25_decoder.hpp`
```cpp
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

    static bool parse_layout(
        const std::vector<int64_t>& shape,
        int& c, int& n, bool& chw_like);

    RawBuffObject decode_one(const float* ptr, int c, const LetterboxMeta& meta) const;
    std::vector<RawBuffObject> nms(const std::vector<RawBuffObject>& in) const;
};

}  // namespace autobuff::detector
```

## 5.5 decoder 通道解释规则（强制）

1. 支持 `[1,C,N]` 和 `[1,N,C]` 自动识别。  
2. 若 `C == 15`，视为 `4 + 1 + 10`（5 keypoints，默认 FAN）。  
3. 若 `C == 17`，视为 `4 + 1 + 12`（6 keypoints，默认 FAN）。  
4. 若 `C >= 19`，视为 `4 + 1 + class + 2*kpt`，使用 class argmax 判定 FAN/R。  
5. 坐标还原：
   - `x = (x_net - pad_x) / scale`
   - `y = (y_net - pad_y) / scale`
   - clamp 到 `[0, src_w/src_h]`

## 5.6 `postprocess.hpp`（从 Raw 到 BuffDetectionResult）
```cpp
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
    int angle_to_slot(double angle) const;
    double calc_angle(const cv::Point2f& p, const cv::Point2f& c) const;
};

}  // namespace autobuff::detector
```

## 5.7 OpenVINO/TensorRT 后端类统一接口

- 类名：`OpenvinoBuffDetector`、`TensorrtBuffDetector`
- 继承：`BuffDetectorInterface`
- 必须实现：
  1. `detect(const cv::Mat&, double)`
  2. `set_enemy_color/get_enemy_color`
  3. `is_async`
  4. `push/pop`（真实异步）

### 5.7.1 `detect()` 固定流程
1. 读取图像并 letterbox。  
2. 推理。  
3. decoder 解析 raw objects。  
4. postprocessor 输出 `BuffDetectionResult`。  
5. 填写 `timestamp/frame_id/robot_state/latency`。

### 5.7.2 `push/pop` 队列规则
1. push 非阻塞，若内部队列满则丢弃最旧帧（保证低延迟）。
2. pop 阻塞等待结果，停止时返回空 sentinel。
3. async 队列长度 Dashboard 上报：`buff_detector.queue_size`。

---

## 6. Observation 层实现（槽位量化 + 去抖 + 实例化）

## 6.1 文件
1. `observation/slot_indexer.hpp/.cpp`
2. `observation/observation_builder.hpp/.cpp`
3. `observation/slot_debouncer.hpp/.cpp`
4. `observation/instance_manager.hpp/.cpp`

## 6.2 `slot_indexer.hpp`
```cpp
namespace autobuff::observation {

class SlotIndexer {
public:
    struct Config { double phase_bias = 0.0; };

    explicit SlotIndexer(Config cfg);
    int angle_to_slot(double angle) const;
    double slot_to_angle(int slot_id) const;

private:
    Config cfg_;
};

}  // namespace autobuff::observation
```

### 6.2.1 量化公式
1. `step = 2*pi/5`
2. `u = normalize(angle - phase_bias)`
3. `slot = round(u / step) mod 5`

## 6.3 `observation_builder.hpp`
```cpp
namespace autobuff::observation {

class ObservationBuilder {
public:
    struct Config {
        double min_pnp_points = 4;
    };

    explicit ObservationBuilder(Config cfg = {});

    BuffFrameObservation build(const autobuff::BuffDetectionResult& det) const;

private:
    Config cfg_;
    SlotIndexer indexer_;

    bool solve_group_pose(const autobuff::BuffDetectionResult& det,
                          Eigen::Vector3d& center_cam,
                          Eigen::Vector3d& normal_cam,
                          std::array<Eigen::Vector3d, autobuff::NUM_SLOTS>& slot_cam) const;
};

}  // namespace autobuff::observation
```

### 6.3.1 `build()` 具体步骤
1. 拷贝 frame/timestamp/state/r_center。
2. 对每个 valid target 生成 `SlotMeasurement`。
3. 调 `solve_group_pose`：
   - object 点：`(R*cos(i*72°), R*sin(i*72°), 0)`
   - image 点：target centers + r_center（如可用）
4. pose 成功则填 `center_cam/normal_cam/slot_cam`。

## 6.4 `slot_debouncer.hpp`
```cpp
namespace autobuff::observation {

class SlotDebouncer {
public:
    struct Config {
        int on_frames = 3;
        int off_frames = 4;
        double missing_timeout = 0.12;
    };

    explicit SlotDebouncer(Config cfg);

    void reset();
    DebouncedBuffObservation update(const BuffFrameObservation& in);

private:
    Config cfg_;
    std::array<SlotTrackState, autobuff::NUM_SLOTS> tracks_{};

    void update_one(int slot, const SlotMeasurement& m, double t,
                    StableSlotObservation& out_slot, bool& state_changed);
};

}  // namespace autobuff::observation
```

### 6.4.1 状态机严格转移表
| state | raw | transition | condition |
|---|---|---|---|
| UNKNOWN/OFF | lit=true | -> CANDIDATE_ON | on_count=1 |
| CANDIDATE_ON | lit=true | keep | on_count++ |
| CANDIDATE_ON | lit=true | -> ON | on_count>=on_frames |
| ON | lit=false | -> CANDIDATE_OFF | off_count=1 |
| CANDIDATE_OFF | lit=false | keep | off_count++ |
| CANDIDATE_OFF | lit=false | -> OFF | off_count>=off_frames |
| 任意 | missing_timeout | -> OFF | t-last_update > timeout |

## 6.5 `instance_manager.hpp`
```cpp
namespace autobuff::observation {

class InstanceManager {
public:
    void reset();
    void stamp(DebouncedBuffObservation& obs);

private:
    std::array<uint32_t, autobuff::NUM_SLOTS> counters_{};
    std::array<bool, autobuff::NUM_SLOTS> last_lit_{};
};

}  // namespace autobuff::observation
```

规则：`last_lit=false && now_lit=true` 时 `counter++` 并写 `instance_id`。

---

## 7. Predictor 层实现（模型化）

## 7.1 文件
1. `predictor/mode_manager.hpp/.cpp`
2. `predictor/models/model_interface.hpp`
3. `predictor/models/const_model.hpp/.cpp`
4. `predictor/models/small_ekf_model.hpp/.cpp`
5. `predictor/models/large_lsm_model.hpp/.cpp`
6. `predictor/buff_predictor.hpp/.cpp`

## 7.2 `mode_manager.hpp`
```cpp
namespace autobuff::predictor {

class ModeManager {
public:
    struct Config {
        int enter_large_active_frames = 3;
        int exit_large_active_frames = 4;
    };

    explicit ModeManager(Config cfg);

    void reset();
    autobuff::BuffMode update(const observation::DebouncedBuffObservation& obs);
    autobuff::BuffMode current() const { return mode_; }

private:
    Config cfg_;
    autobuff::BuffMode mode_ = autobuff::BuffMode::UNKNOWN;
    int active_streak_ = 0;
    int inactive_streak_ = 0;
};

}  // namespace autobuff::predictor
```

## 7.3 `model_interface.hpp`
```cpp
namespace autobuff::predictor::models {

class MotionModelInterface {
public:
    virtual ~MotionModelInterface() = default;
    virtual void reset() = 0;
    virtual void feed(const observation::DebouncedBuffObservation& obs, int track_slot) = 0;
    virtual MotionEstimate estimate() const = 0;
    virtual double delta_theta(double t_abs, double dt) const = 0;
};

}  // namespace autobuff::predictor::models
```

## 7.4 `const_model.hpp`
```cpp
namespace autobuff::predictor::models {

class ConstModel final : public MotionModelInterface {
public:
    void reset() override;
    void feed(const observation::DebouncedBuffObservation& obs, int track_slot) override;
    MotionEstimate estimate() const override;
    double delta_theta(double t_abs, double dt) const override;

private:
    int dir_ = 1;
    double omega_ = M_PI / 3.0;
    bool inited_ = false;

    double last_phi_ = 0.0;
    double last_t_ = 0.0;
    int dir_votes_ = 0;
};

}  // namespace autobuff::predictor::models
```

## 7.5 `small_ekf_model.hpp`
```cpp
namespace autobuff::predictor::models {

class SmallEkfModel final : public MotionModelInterface {
public:
    void reset() override;
    void feed(const observation::DebouncedBuffObservation& obs, int track_slot) override;
    MotionEstimate estimate() const override;
    double delta_theta(double t_abs, double dt) const override;

private:
    aimer::filter::AdaptiveEkf<2, 1> ekf_;
    bool ekf_inited_ = false;

    int dir_ = 1;
    int dir_votes_ = 0;

    double last_phi_ = 0.0;
    double last_t_ = 0.0;
    bool has_last_ = false;

    static double normalize_angle(double x);
    static double unwrap_to_near(double meas, double ref);

    void update_dir(double phi, double dt);
};

}  // namespace autobuff::predictor::models
```

### 7.5.1 Small EKF 数学定义
1. 状态：`x=[phi, omega]^T`。
2. 预测：
   - `phi_k+1 = phi_k + omega_k*dt`
   - `omega_k+1 = omega_k`
3. 观测：`z = phi_meas`。
4. 噪声：从 `AutoBuff.Predictor.SmallEKF.*` 读取。
5. 软约束：将 `omega_nominal = dir*pi/3` 作为先验引导，不做硬固定。

## 7.6 `large_lsm_model.hpp`
```cpp
namespace autobuff::predictor::models {

class LargeLsmModel final : public MotionModelInterface {
public:
    void reset() override;
    void feed(const observation::DebouncedBuffObservation& obs, int track_slot) override;
    MotionEstimate estimate() const override;
    double delta_theta(double t_abs, double dt) const override;

private:
    struct Sample { double t_rel = 0.0; double phi = 0.0; };

    std::deque<Sample> samples_;
    bool active_ = false;

    double start_time_ = 0.0;
    double last_phi_raw_ = 0.0;
    double phi_unwrap_ = 0.0;
    bool has_last_phi_ = false;

    int dir_ = 1;
    int dir_votes_ = 0;
    double last_t_ = 0.0;

    LargeSineParam param_{};

    static double normalize_angle(double x);
    void update_dir(double phi_raw, double t);
    void push_sample(double t_abs, double phi_raw);
    void trim_window(double window_sec);
    bool solve_lsq();
    double phi_model(double t_rel) const;
};

}  // namespace autobuff::predictor::models
```

### 7.6.1 LSM 优化目标（强制）
最小化：
`sum_i rho( phi_pred(t_i; a,w,tau,phi0,dir) - phi_meas_i )`

其中：
`phi_pred = dir * (-(a/w)*cos(w*(t_i+tau)) + (2.090-a)*t_i) + phi0`

边界：
- `a in [0.780, 1.045]`
- `w in [1.884, 2.000]`
- `tau in [-0.5, 0.5]`

失败处理：
1. `summary.IsSolutionUsable()==false` -> invalid。
2. `residual_rms > residual_accept` -> invalid。
3. invalid 时上层回退 `ConstModel`。

## 7.7 `buff_predictor.hpp`
```cpp
namespace autobuff::predictor {

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

}  // namespace autobuff::predictor
```

### 7.7.1 `predict()` 固定流程
1. `obs = obs_builder_.build(det)`。
2. `debounced = debouncer_.update(obs)`。
3. `instance_mgr_.stamp(debounced)`。
4. `mode = mode_mgr_.update(debounced)`。
5. `track_slot = choose_track_slot(debounced)`。
6. 模式分发：
   - `SMALL_ACTIVE`: `small_model.feed`。
   - `LARGE_ACTIVE`: `large_model.feed`；若 `estimate.invalid` -> `const_model`。
   - `LARGE_INACTIVE`: `const_model.feed`。
7. 组装 `BuffSnapshot`。
8. 计算 `recommended_slot` 与 `ccw_rank_to_slot`。
9. 返回 snapshot。

---

## 8. FireControl 层实现（协同 + 门控）

## 8.1 文件
1. `fire_control/types.hpp`
2. `fire_control/target_ranker.hpp/.cpp`
3. `fire_control/coop_policy.hpp/.cpp`
4. `fire_control/fire_controller.hpp/.cpp`

## 8.2 `target_ranker.hpp`
```cpp
namespace autobuff::fire_control {

class TargetRanker {
public:
    explicit TargetRanker(const FireControlConfig& cfg);

    std::vector<SlotAimCandidate> build(
        const predictor::BuffSnapshot& snap,
        const ::fire_control::LatencyInfo& latency,
        const ::fire_control::GimbalState& gimbal) const;

private:
    FireControlConfig cfg_;

    static double normalize_angle(double x);
    static double tracking_error(const ::fire_control::AimResult& aim,
                                 const ::fire_control::GimbalState& gimbal);
};

}  // namespace autobuff::fire_control
```

### 8.2.1 候选构建规则
1. 只从 `lit slots` 构建候选；无 lit 时回退 valid slots。
2. 对每个候选：
   - `impact_dt = latency.prediction_latency()`
   - `pred_world = snapshot.predict_slot_world(slot, impact_dt)`
   - `aim = trajectory::solve(pred_world, bullet_speed)`
   - `tracking_error = hypot(dyaw, dpitch)`
3. 打分：
`score = -score_w_error * tracking_error + score_w_conf * confidence`

## 8.3 `coop_policy.hpp`
```cpp
namespace autobuff::fire_control {

class CoopPolicy {
public:
    explicit CoopPolicy(const FireControlConfig& cfg);

    int select(
        const predictor::BuffSnapshot& snap,
        const std::vector<SlotAimCandidate>& cands,
        bool& coop_applied) const;

private:
    FireControlConfig cfg_;
};

}  // namespace autobuff::fire_control
```

### 8.3.1 协同选择规则
1. 若 `coop_role == DISABLED` -> 返回 `best score`。
2. 若 `coop_only_large_active` 且 `mode != LARGE_ACTIVE` -> 返回 `best score`。
3. 否则：
   - `CCW_FIRST`：优先 `rank=0`
   - `CCW_SECOND`：优先 `rank=1`，缺失时降级 `rank=0`
4. 指定 rank 若无弹道可解候选，回退 best score。

## 8.4 `fire_controller.hpp`
```cpp
namespace autobuff::fire_control {

class FireController {
public:
    FireController();

    void reset();

    ::fire_control::FireCommand control(
        const predictor::BuffSnapshot& snapshot,
        double current_time,
        const ::fire_control::LatencyInfo& latency);

private:
    FireControlConfig cfg_;

    TargetRanker ranker_;
    CoopPolicy coop_;

    ::fire_control::GimbalState gimbal_state_;
    double last_time_ = 0.0;
    int lost_count_ = 0;

    std::unordered_map<uint64_t, double> fired_stamp_;

    static uint64_t fire_key(int slot, uint32_t instance_id);

    bool allow_refire(int slot, uint32_t instance_id, double now) const;
    void mark_fired(int slot, uint32_t instance_id, double now);

    bool can_fire_gate(const SlotAimCandidate& c,
                       const predictor::BuffSnapshot& snap,
                       double prediction_dt) const;

    ::fire_control::FireCommand no_target_command() const;
};

}  // namespace autobuff::fire_control
```

### 8.4.1 `control()` 固定流程
1. 更新 `gimbal_state`。
2. snapshot 无效 -> no target。
3. `cands = ranker_.build(...)`。
4. `chosen_slot = coop_.select(...)`。
5. 对 chosen 做门控：
   - `confidence >= min_confidence`
   - `tracking_error <= fire_threshold`
   - `prediction_dt <= max_prediction_latency`
   - `allow_fire` 置位
   - `allow_refire` true
6. 输出 `FireCommand`（兼容字段）。
7. 若触发开火，调用 `mark_fired`。

---

## 9. Node 层重构细节

## 9.1 `detector_node.cpp`
1. 保持 sync/async 双模式。
2. 非能量机关模式继续跳过。
3. 发布 topic 保持 `buff_detections`。
4. dashboard 新增：
   - `buff_detector.backend`
   - `buff_detector.queue_size`
   - `buff_detector.lit_count`

## 9.2 `predictor_node.cpp`
1. 保持订阅 `buff_detections`。
2. 输出 `buff_snapshot` 维持 `BasicObjManager`。
3. dashboard 新增：
   - `buff_predictor.mode`
   - `buff_predictor.model`
   - `buff_predictor.fit_rms`
   - `buff_predictor.recommended_slot`

## 9.3 `fire_control_node.cpp`
1. 保持 500Hz 循环。
2. 保持 `LatencyEstimator` 逻辑。
3. `finalize_latency` 继续迭代 `fire_to_hit`。
4. dashboard 新增：
   - `buff_fire.slot`
   - `buff_fire.rank`
   - `buff_fire.error`
   - `buff_fire.can_fire`

---

## 10. 配置文件完整增量

## 10.1 `config/buff.toml`（替换 YoloDetector 段）
```toml
[Detector]
    type = "yolo"
    debug = false

[Detector.yolo]
    backend = "openvino"          # openvino / tensorrt
    model_path = "buff_25_best.onnx"
    input_size = 640
    confidence_threshold = 0.45
    nms_threshold = 0.45
    max_det = 64

[Detector.yolo.decode]
    decoder = "sp25"
    class_num = 2
    fan_class_id = 0
    r_class_id = 1
    kpt_max = 8
    kpt_center_idx = 4
    kpt_r_hint_idx = 5

[Detector.yolo.postprocess]
    lit_thresh = 0.50
    min_target_conf = 0.35
    min_r_conf = 0.35
    slot_phase_bias = 0.0
```

## 10.2 `config/aimer.toml`（新增段）
```toml
[AutoBuff.Observation]
    on_frames = 3
    off_frames = 4
    missing_timeout = 0.12

[AutoBuff.Mode]
    enter_large_active_frames = 3
    exit_large_active_frames = 4

[AutoBuff.Predictor.SmallEKF]
    q_phi = 0.0002
    q_omega = 0.005
    r_phi = 0.004
    omega_nominal = 1.0471975512
    omega_prior_sigma = 0.20

[AutoBuff.Predictor.LargeLSM]
    window_sec = 2.2
    min_samples = 35
    min_span_sec = 0.6
    huber_delta = 0.1
    a_min = 0.780
    a_max = 1.045
    w_min = 1.884
    w_max = 2.000
    tau_bound = 0.5
    residual_accept = 0.18

[AutoBuff.FireControl]
    fire_threshold = 0.02
    min_confidence = 0.30
    max_prediction_latency = 0.35
    refire_block_sec = 0.08
    score_w_error = 1.0
    score_w_conf = 0.2
    coop_role = "CCW_FIRST"         # DISABLED / CCW_FIRST / CCW_SECOND
    coop_only_large_active = true

[AutoBuff.FireControl.Latency]
    send_to_control = 0.003
    control_to_fire = 0.020
```

---

## 11. 关键算法伪代码（实现不可偏离）

## 11.1 Detector 后处理
```cpp
objects = decoder.decode(output, shape, letterbox_meta)
r_center = choose_r_center(objects)
for obj in fan_objects:
    angle = atan2(-(obj.center.y - r_center.y), obj.center.x - r_center.x)
    slot = angle_to_slot(angle)
    if slot_conflict: keep higher confidence
    target[slot] = ...
result.recompute_summary()
```

## 11.2 Debounce
```cpp
for slot in 0..4:
    if missing_timeout: force OFF
    else switch(track.state):
        OFF/UNKNOWN + lit -> CANDIDATE_ON
        CANDIDATE_ON + lit_count>=on_frames -> ON
        ON + !lit -> CANDIDATE_OFF
        CANDIDATE_OFF + off_count>=off_frames -> OFF
if OFF->ON: state_changed=true
```

## 11.3 Large LSM
```cpp
if not large_active: return
push (t_rel, phi_unwrapped)
trim window
if samples < min_samples or span < min_span: return
solve ceres with bounds
if !usable or rms > threshold: param.valid=false
else param.valid=true
```

## 11.4 FireControl 协同
```cpp
cands = build_candidates(snapshot)
if coop enabled and mode allowed:
    prefer rank slot by role
    if unavailable -> best score
else:
    best score
fire_now = all gates pass && refire_block passed
```

---

## 12. 误差与容错策略（必须实现）

1. `r_center` 缺失：本帧 `status=TARGETS_ONLY`，预测层可延用上帧中心一次（最多 1 帧）。
2. PnP 失败：`DebouncedBuffObservation.valid=false`，predictor 输出 invalid snapshot。
3. 大符拟合失败：降级常速模型，不中断 fire_control。
4. 队列拥堵：detector async 模式丢旧帧，不堆积延迟。
5. 跳模瞬间：强制清空 large samples，避免错误参数跨模式继承。

---

## 13. 日志与可观测性

### 13.1 必加 dashboard 键
1. `buff_detector.backend`
2. `buff_detector.latency_ms`
3. `buff_detector.lit_count`
4. `buff_predictor.mode`
5. `buff_predictor.model`
6. `buff_predictor.omega`
7. `buff_predictor.fit_rms`
8. `buff_fire.selected_slot`
9. `buff_fire.selected_rank`
10. `buff_fire.tracking_error`
11. `buff_fire.fire_now`

### 13.2 必加日志点
1. mode 切换：`UNKNOWN->SMALL_ACTIVE` 等。
2. large fit 成功/失败（打印 `a,w,tau,rms,sample_count`）。
3. 协同策略选中 `rank/slot`。
4. refire 被抑制事件。

---

## 14. 测试计划（文件与阈值）

## 14.1 单测列表
1. `test/auto_buff/test_sp25_decoder.cpp`
2. `test/auto_buff/test_slot_indexer.cpp`
3. `test/auto_buff/test_slot_debouncer.cpp`
4. `test/auto_buff/test_instance_manager.cpp`
5. `test/auto_buff/test_small_ekf_model.cpp`
6. `test/auto_buff/test_large_lsm_model.cpp`
7. `test/auto_buff/test_coop_policy.cpp`
8. `test/auto_buff/test_fire_controller_gate.cpp`

## 14.2 验收阈值
1. slot 去抖抖动率 `< 3%`。
2. small 模式 `|omega - pi/3| < 0.08 rad/s`（稳定段）。
3. large 拟合合成数据 `|a_err|<0.08`, `|w_err|<0.05`, `|tau_err|<0.12s`。
4. 协同模式在双亮场景 FIRST/SECOND 正确率 `> 98%`。
5. OV/TRT 推荐槽位一致率 `> 95%`（同回放）。

---

## 15. 分阶段提交建议（给实现者）

1. **PR-1 类型与目录骨架**：新增类型、空实现、CMake 连通。  
2. **PR-2 detector core + OV**：先打通 openvino。  
3. **PR-3 TRT backend**：接入 tensorrt。  
4. **PR-4 observation 层**：去抖 + 实例化。  
5. **PR-5 predictor 模型**：small ekf + large lsm。  
6. **PR-6 fire_control 协同**：候选排序、角色策略、门控。  
7. **PR-7 测试与参数整定**：单测 + 回放。

---

## 16. 实施检查清单（DoD）

1. [ ] `Detector.type=yolo` 时可按 backend 正常起推理。  
2. [ ] `buff_detections` 可稳定输出 `slot_id/lit_mask`。  
3. [ ] 预测层输出 `mode/motion/recommended_slot/ccw_rank`。  
4. [ ] fire_control 在 `DISABLED/FIRST/SECOND` 角色下行为可复现。  
5. [ ] 协议兼容：`FireCommand` 字段不变。  
6. [ ] 所有新增 runtime 参数支持热更新读取。  
7. [ ] 单测与回放测试通过并达到阈值。

