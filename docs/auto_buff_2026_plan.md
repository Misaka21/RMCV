# AutoBuff 2026 完整重构方案

> 本文件是 AutoBuff 模块推翻重做的完整实现规格，供 Codex / Claude Code 直接落地。
>
> **核心目标**：
> 1. 用 sp_vision_25 的 YOLO11 关键点模型替换传统检测器
> 2. 支持 OpenVINO 和 TensorRT 两种推理后端
> 3. 小符用 EKF 滤波，大符用最小二乘拟合
> 4. 实现双车协同打符（逆时针第1个 / 第2个）
> 5. 代码风格与现有 auto_aim 保持一致

---

## 1. 比赛规则约束 (2025/2026)

### 1.1 能量机关物理结构
- 5 个扇叶围绕中心 R 标均匀分布，间距 72°
- R 标到靶心中心距离: **700mm**
- 红蓝两侧各一个，共轴反向旋转
- 每场比赛旋转方向随机，**整场保持不变**

### 1.2 速度规则
| 模式 | 速度 | 说明 |
|------|------|------|
| 小符 (ENERGY_SMALL) | `ω = π/3 rad/s` (60°/s) | 恒定速度 |
| 大符非激活 (ENERGY_LARGE, lit<2) | `ω = π/3 rad/s` | 与小符相同 |
| 大符激活 (ENERGY_LARGE, lit≥2) | `spd(t) = a·sin(w·t) + b` | 正弦变速 |

大符激活参数范围：
- `a ∈ [0.780, 1.045]`
- `w ∈ [1.884, 2.000]` (rad/s)
- `b = 2.090 - a`
- 每次进入激活态，参数重新随机，时间原点重置
- 实际速度相对目标函数存在 ≤500ms 时间偏差

### 1.3 激活规则
- 小符激活时：**1 块亮**（可打）
- 大符激活时：**2 块亮**（可打）
- 击中亮的扇叶后该扇叶灭，下一个亮起

### 1.4 内部模式定义
```cpp
enum class BuffMode : uint8_t {
    UNKNOWN = 0,        // 非能量机关模式
    SMALL_ACTIVE = 1,   // 小符 (1块亮)
    LARGE_INACTIVE = 2, // 大符非激活 (0块亮，恒速)
    LARGE_ACTIVE = 3,   // 大符激活 (2块亮，变速)
};
```

模式判定：
1. `aim_mode == ENERGY_SMALL` → `SMALL_ACTIVE`
2. `aim_mode == ENERGY_LARGE` 且 `lit_count >= 2` 连续 N 帧 → `LARGE_ACTIVE`
3. `aim_mode == ENERGY_LARGE` 且 `lit_count < 2` 连续 N 帧 → `LARGE_INACTIVE`
4. 非能量机关模式 → `UNKNOWN`，重置所有预测状态

---

## 2. 总体数据流

```
Hardware (200Hz)
    │
    ▼
SyncFrame { image, serial_data, timestamp }
    │
    ▼ (模式过滤: ENERGY_SMALL / ENERGY_LARGE)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Detector Thread (buff_detector_node)
    │  YOLO11 推理 (OV/TRT)
    │  ↓ Sp25Decoder 解析 6关键点
    │  ↓ R标定位 (从kpt[5]推导 + 轮廓精修)
    │  ↓ 角度量化到 slot_id (0~4)
    │  ↓ 被检测到 = lit (模型只检测可见/亮的扇叶)
    ▼
Message<BuffDetectionResult>("buff_detections")
    │
    ▼
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Predictor Thread (buff_predictor_node)
    │  ① ObservationBuilder: PnP → 3D中心/法向量
    │  ② SlotDebouncer: 去抖状态机 (防检测抖动)
    │  ③ ModeManager: 判定 SMALL/LARGE_INACTIVE/LARGE_ACTIVE
    │  ④ DirectionEstimator: 投票确定 CW/CCW (全模型共享)
    │  ⑤ 模型分发:
    │     ├─ SMALL_ACTIVE → SmallEkfModel (AdaptiveEkf<2,1>)
    │     ├─ LARGE_INACTIVE → ConstModel (ω=π/3)
    │     └─ LARGE_ACTIVE → LargeLsmModel (Ceres拟合)
    │         └─ 拟合失败 → 降级 ConstModel
    │  ⑥ 组装 BuffSnapshot + CCW排序
    ▼
BasicObjManager<BuffSnapshot>("buff_snapshot")
    │
    ▼
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
FireControl Thread (500Hz)
    │  ① 候选构建: lit slots → 弹道解算 → tracking_error
    │  ② 协同策略: CCW_FIRST/CCW_SECOND → 选目标
    │  ③ 开火门控: error < threshold && confidence >= min
    ▼
BasicObjManager<FireCommand>("fire_command")
    │
    ▼
Serial 发送
```

### 2.1 线程拓扑 (保持现有 main.cpp)
```cpp
// main.cpp 中的启动顺序
autobuff::detector::background_buff_detector_run("buff.toml");
autobuff::predictor::start_predictor_node();
autobuff::fire_control::start_fire_control_node("aimer.toml");
```

### 2.2 UMT 通道契约
| 通道 | 类型 | 方向 |
|------|------|------|
| `"buff_detections"` | `Message<BuffDetectionResult>` | Detector → Predictor |
| `"buff_snapshot"` | `BasicObjManager<BuffSnapshot>` | Predictor → FireControl |
| `"fire_command"` | `BasicObjManager<FireCommand>` | FireControl → Serial |
| `"/buff_detector/debug"` | `Publisher<cv::Mat>` | Detector → Web |

### 2.3 时间戳约束
- 全流程使用 `steady_clock` 秒值
- `BuffDetectionResult.timestamp` = 图像帧时刻
- `BuffSnapshot.predict_timestamp` = predictor 处理完成时刻
- FireControl 用 `LatencyEstimator.update_predict_to_send(now - predict_timestamp)` 估计延迟

---

## 3. 目录结构与 CMake 目标

```
aimer/auto_buff/
├── common/
│   └── types.hpp                          # [修改] 增加 BuffMode/RotateDir/CoopRole 枚举
│
├── detector/
│   ├── common/
│   │   ├── detector_interface.hpp         # [保持]
│   │   ├── raw_types.hpp                  # [新增] LetterboxMeta, RawBuffObject
│   │   ├── preprocess.hpp/.cpp            # [新增] letterbox + HWC→CHW
│   │   └── postprocess.hpp/.cpp           # [新增] Raw → BuffDetectionResult
│   ├── decoder/
│   │   ├── buff_decoder.hpp               # [新增] IBuffDecoder 接口
│   │   └── sp25_decoder.hpp/.cpp          # [新增] sp_vision_25 模型解码器
│   ├── detector_ov/
│   │   └── openvino_detector.hpp/.cpp     # [新增] OpenVINO 推理后端
│   ├── detector_trt/
│   │   └── tensorrt_detector.hpp/.cpp     # [新增] TensorRT 推理后端
│   ├── traditional/                       # [保持] 传统检测器
│   ├── detector_factory.hpp/.cpp          # [修改] 增加 yolo 分支
│   ├── detector_node.hpp/.cpp             # [保持]
│   └── CMakeLists.txt                     # [修改]
│
├── predictor/
│   ├── types.hpp                          # [修改] 增加 MotionEstimate, ccw_rank
│   ├── observation_builder.hpp/.cpp       # [新增] 检测结果 → 3D观测
│   ├── slot_debouncer.hpp/.cpp            # [新增] 去抖状态机
│   ├── direction_estimator.hpp/.cpp       # [新增] 方向投票 (集中管理)
│   ├── mode_manager.hpp/.cpp              # [新增] 模式判定
│   ├── models/
│   │   ├── model_interface.hpp            # [新增] MotionModelInterface
│   │   ├── const_model.hpp/.cpp           # [新增] 恒速模型
│   │   ├── small_ekf_model.hpp/.cpp       # [新增] 小符 EKF
│   │   └── large_lsm_model.hpp/.cpp       # [新增] 大符最小二乘
│   ├── buff_predictor.hpp/.cpp            # [重写]
│   ├── predictor_node.hpp/.cpp            # [修改]
│   └── CMakeLists.txt                     # [修改]
│
├── fire_control/
│   ├── types.hpp                          # [新增] SlotAimCandidate, CoopPolicy 类型
│   ├── target_ranker.hpp/.cpp             # [新增] 候选排序
│   ├── coop_policy.hpp/.cpp               # [新增] 双车协同策略
│   ├── fire_controller.hpp/.cpp           # [重写]
│   ├── fire_control_node.hpp/.cpp         # [修改]
│   └── CMakeLists.txt                     # [修改]
│
└── CMakeLists.txt                         # [修改]
```

### 3.1 CMake 目标依赖
```
buff_detector_core     → auto_buff_common, aimer_common, OpenCV
buff_detector_openvino → buff_detector_core, OpenVINO    (可选, ifdef ENABLE_OPENVINO_DETECTOR)
buff_detector_tensorrt → buff_detector_core, TensorRT    (可选, ifdef ENABLE_TENSORRT_DETECTOR)
buff_detector_node     → buff_detector_core + 后端库, hardware
buff_predictor         → auto_buff_common, aimer_common, Ceres
buff_fire_control      → buff_predictor, aimer_common
```

---

## 4. 类型系统 (逐头文件)

### 4.1 `common/types.hpp` — 公共枚举与检测结构

在现有基础上**新增**以下枚举（保留已有的 `DetectionStatus`, `DetectedRCenter`, `DetectedTarget`, `BuffDetectionResult`）：

```cpp
// ===== 新增枚举 =====

enum class BuffMode : uint8_t {
    UNKNOWN = 0,
    SMALL_ACTIVE = 1,
    LARGE_INACTIVE = 2,
    LARGE_ACTIVE = 3,
};

enum class RotateDir : int8_t {
    UNKNOWN = 0,
    CW = -1,    // 顺时针
    CCW = 1,    // 逆时针
};

// 双车协同角色
enum class CoopRole : uint8_t {
    DISABLED = 0,     // 不协同，打最优目标
    CCW_FIRST = 1,    // 打逆时针方向第 1 个 lit slot
    CCW_SECOND = 2,   // 打逆时针方向第 2 个 lit slot
};

// 推理后端
enum class DetectorBackend : uint8_t {
    TRADITIONAL = 0,
    OPENVINO = 1,
    TENSORRT = 2,
};
```

**修改 `DetectedTarget`**：
```cpp
struct DetectedTarget {
    cv::Point2f center{};

    // 6 个关键点 (从 sp25 模型):
    // kpt[0-3]: 扇叶四角 (左上逆时针)
    // kpt[4]:   扇叶中心
    // kpt[5]:   内侧尖端 (指向R标方向)
    std::array<cv::Point2f, 6> keypoints{};
    uint8_t keypoint_count = 0;

    int slot_id = -1;
    double angle = 0.0;   // 相对 R 标的角度 (rad)

    // ★ 关键: 被模型检测到 = lit (模型只检测亮着的扇叶)
    // is_lit 由后处理根据检测结果设置，不依赖模型分类
    bool is_lit = false;
    bool valid = false;
    float confidence = 0.f;
};
```

**修改 `BuffDetectionResult`**：增加 `backend` 字段
```cpp
struct BuffDetectionResult {
    DetectorBackend backend = DetectorBackend::TRADITIONAL;
    // ... 其余保持不变
};
```

### 4.2 `detector/common/raw_types.hpp` — 推理原始输出

```cpp
namespace autobuff::detector {

struct LetterboxMeta {
    int src_w = 0, src_h = 0;
    int net_w = 640, net_h = 640;
    float scale = 1.f;
    float pad_x = 0.f, pad_y = 0.f;
};

struct RawBuffObject {
    cv::Rect2f box{};
    float score = 0.f;

    // 最多 6 个关键点 (sp25 模型固定 6 个)
    std::array<cv::Point2f, 6> kpts{};
    uint8_t kpt_count = 0;
};

}  // namespace autobuff::detector
```

> **注意**: 不需要 `RawClassId` 枚举。sp25 模型是单类检测器，所有检测都是 fan blade。
> R 标通过 keypoint[5] 推导，不是模型分类结果。

### 4.3 `predictor/types.hpp` — 预测层输出

```cpp
namespace autobuff::predictor {

enum class SpeedModel : uint8_t {
    UNKNOWN = 0,
    CONST_OMEGA = 1,      // 恒速 (小符 / 大符非激活)
    LARGE_SINE_LSM = 2,   // 正弦拟合 (大符激活)
};

struct LargeSineParam {
    bool valid = false;
    int dir = 1;              // +1 CCW, -1 CW
    double start_time = 0.0;  // t=0 绝对时间戳

    double a = 0.90;          // [0.780, 1.045]
    double w = 1.94;          // [1.884, 2.000]
    double tau = 0.0;         // 时间偏移 [-0.5, 0.5]
    double phi0 = 0.0;        // 初始相位偏移

    double residual_rms = 1e9;
    int sample_count = 0;

    double b() const { return 2.090 - a; }

    // 累积角度: ∫spd(t)dt = -(a/w)*cos(w*(t+tau)) + b*t
    double phi(double t_rel) const {
        return static_cast<double>(dir)
             * (-(a / w) * std::cos(w * (t_rel + tau)) + b() * t_rel)
             + phi0;
    }

    double delta(double t_rel, double dt) const {
        return phi(t_rel + dt) - phi(t_rel);
    }
};

// 运动估计结果 (模型输出)
struct MotionEstimate {
    SpeedModel model = SpeedModel::UNKNOWN;
    double omega_signed = 0.0;   // CONST_OMEGA 模式: 带符号角速度
    LargeSineParam large{};      // LARGE_SINE_LSM 模式
    double confidence = 0.0;

    // 统一的 delta_theta 计算 (火控插值用)
    double delta_theta(double t_abs, double dt) const {
        if (model == SpeedModel::LARGE_SINE_LSM && large.valid) {
            double t_rel = t_abs - large.start_time;
            return large.delta(t_rel, dt);
        }
        return omega_signed * dt;
    }
};

struct RuneSlotState {
    bool valid = false;
    bool is_lit = false;
    float confidence = 0.f;

    cv::Point2f center_px{};
    double angle = 0.0;  // 相对 R 标角度 (rad)

    Eigen::Vector3d pos_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_cam = Eigen::Vector3d::Zero();  // center→slot 向量 (相机系)
};

struct BuffSnapshot {
    bool valid = false;
    int frame_id = 0;
    double timestamp = 0.0;
    double predict_timestamp = 0.0;

    aimer::RobotState self_state{};
    autobuff::BuffMode mode = autobuff::BuffMode::UNKNOWN;
    autobuff::RotateDir direction = autobuff::RotateDir::UNKNOWN;

    // 运动估计
    MotionEstimate motion{};

    // 旋转中心与平面法向 (相机系/世界系)
    Eigen::Vector3d center_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_cam = Eigen::Vector3d(0, 0, 1);

    // 5 个槽位状态
    std::array<RuneSlotState, autobuff::NUM_SLOTS> slots{};
    uint8_t lit_mask = 0;
    int lit_count = 0;
    int recommended_slot = -1;

    // ★ 双车协同: 逆时针排序 (rank[0]=逆时针第1个lit, rank[1]=第2个lit, ...)
    std::array<int, autobuff::NUM_SLOTS> ccw_lit_rank{{-1,-1,-1,-1,-1}};
    int ranked_count = 0;

    // === 辅助方法 ===
    bool has_slot(int i) const;
    bool slot_lit(int i) const;

    // ★ 旋转预测 (火控 500Hz 插值用)
    Eigen::Vector3d predict_slot_cam(int slot_id, double dt) const {
        if (!valid || !has_slot(slot_id)) return Eigen::Vector3d::Zero();
        double delta = motion.delta_theta(timestamp, dt);
        Eigen::AngleAxisd aa(delta, normal_cam.normalized());
        return center_cam + aa.toRotationMatrix() * slots[slot_id].vec_cam;
    }

    Eigen::Vector3d predict_slot_world(int slot_id, double dt) const {
        Eigen::Vector3d p = predict_slot_cam(slot_id, dt);
        if (p.isZero(0)) return Eigen::Vector3d::Zero();
        return aimer::tf::cam_to_world(p, self_state.q_imu);
    }
};
```

### 4.4 `fire_control/types.hpp` — 火控内部类型

```cpp
namespace autobuff::fire_control {

struct SlotAimCandidate {
    int slot_id = -1;
    int ccw_rank = -1;   // 逆时针排序位置 (0=第1个)

    bool is_lit = false;
    bool ballistic_valid = false;

    double tracking_error = 1e9;
    float confidence = 0.f;
    double score = -1e9;

    Eigen::Vector3d pred_world = Eigen::Vector3d::Zero();
    ::fire_control::AimResult aim{};
};

}  // namespace autobuff::fire_control
```

---

## 5. Detector 详细实现

### 5.1 模型信息 (sp_vision_25 的 best.onnx)

| 项目 | 值 |
|------|-----|
| 输入 | `[1, 3, 640, 640]` NCHW float32 |
| 输出 | `[1, C, N]`, C=17, N=8400 (或 C=15, N=8400) |
| 通道布局 | `[cx, cy, w, h, score, kp0_x, kp0_y, ..., kp5_x, kp5_y]` |
| 类别数 | **1** (单类: fan blade) |
| 关键点 | 6 个 (无 visibility channel) |
| 置信度阈值 | 0.45 |
| NMS 阈值 | 0.45 |

**6 个关键点的物理含义**：
| Index | 含义 | 用途 |
|-------|------|------|
| kpt[0] | 扇叶角点1 (左上, 逆时针) | PnP 解算 |
| kpt[1] | 扇叶角点2 | PnP 解算 |
| kpt[2] | 扇叶角点3 | PnP 解算 |
| kpt[3] | 扇叶角点4 | PnP 解算 |
| kpt[4] | 扇叶中心 | 角度计算 / 目标定位 |
| kpt[5] | 内侧尖端 (指向 R 标) | R 标位置推导 |

**3D 物理坐标** (PnP 用, 单位 m, 能量机关坐标系):
```cpp
constexpr std::array<cv::Point3f, 6> BLADE_OBJECT_POINTS = {{
    {0.f, 0.f,    0.827f},  // kpt[0] 顶部
    {0.f, 0.127f, 0.700f},  // kpt[1] 右侧
    {0.f, 0.f,    0.573f},  // kpt[2] 底部
    {0.f, -0.127f,0.700f},  // kpt[3] 左侧
    {0.f, 0.f,    0.700f},  // kpt[4] 扇叶中心
    {0.f, 0.f,    0.220f},  // kpt[5] R 标区域
}};
```

### 5.2 Sp25Decoder — 模型输出解码器

```cpp
class IBuffDecoder {
public:
    virtual ~IBuffDecoder() = default;
    virtual std::vector<RawBuffObject> decode(
        const float* data,
        const std::vector<int64_t>& shape,
        const LetterboxMeta& meta) const = 0;
};

class Sp25Decoder final : public IBuffDecoder {
    // 配置
    float conf_thres = 0.45f;
    float nms_thres = 0.45f;
    int max_det = 64;

    std::vector<RawBuffObject> decode(...) const override;
};
```

**decode() 流程**:
1. 自动识别 `[1,C,N]` vs `[1,N,C]`: 若 `dim[1] < dim[2]`，则 `C=dim[1], N=dim[2]` (CHW-like)
2. 通道解读:
   - C==15: 5 keypoints (`4 + 1 + 5*2`)
   - C==17: 6 keypoints (`4 + 1 + 6*2`)
   - 无 class logits (单类)
3. 遍历 N 个候选:
   - `score = data[4*N + i]`，低于 `conf_thres` 跳过
   - 提取 bbox `[cx, cy, w, h]`
   - 提取 keypoints
4. 坐标还原: `x = (x_net - pad_x) / scale`, `y = (y_net - pad_y) / scale`
5. NMS 去重
6. 返回 `vector<RawBuffObject>`

### 5.3 Postprocessor — Raw → BuffDetectionResult

```cpp
class Postprocessor {
public:
    BuffDetectionResult build_result(
        const std::vector<RawBuffObject>& objs,
        const cv::Mat& image, ...) const;
};
```

**build_result() 流程**:
1. **R 标定位** (★ 关键修正: 不依赖模型分类):
   - 从所有检测到的 fan blade 的 `kpt[4]`(中心) 和 `kpt[5]`(内侧) 推导 R 位置
   - 公式: `r_estimate += (kpt[5] - kpt[4]) * 1.4 + kpt[4]` (取所有检测的平均)
   - 可选精修: 在估计位置附近做灰度阈值+轮廓分析找最圆/最方形区域
2. **角度计算**: 对每个 fan blade
   - `angle = atan2(-(center.y - r_center.y), center.x - r_center.x)` (像素坐标, y轴向下取反)
3. **槽位分配**:
   - `slot_id = round(angle / (2π/5)) mod 5`
   - 冲突时保留 confidence 更高的
4. **Lit 判定** (★ 关键修正):
   - **被模型检测到 = lit** (模型只检测亮着的扇叶)
   - 所有 valid target 的 `is_lit = true`
   - 未被检测到的 slot 不填充 (保持 `valid = false`)
5. 调用 `result.update_summary()` 更新统计

### 5.4 OpenvinoBuffDetector

```cpp
class OpenvinoBuffDetector final : public BuffDetectorInterface {
public:
    BuffDetectionResult detect(const cv::Mat& image, double timestamp) override;
    bool is_async() const override { return true; }
    void push(...) override;      // 真正异步推理
    AsyncBuffDetectionResult pop() override;
    // ...
private:
    ov::Core core_;
    ov::CompiledModel compiled_;
    ov::InferRequest infer_;      // 同步模式用
    std::unique_ptr<IBuffDecoder> decoder_;
    Postprocessor post_;
};
```

**detect() 统一流程**:
1. `letterbox_bgr_u8(image, 640, meta)` → 640×640 图像
2. `bgr_to_chw_f32(resized, tensor_data, true, true)` → NCHW float32 RGB [0,1]
3. OpenVINO 推理 → 输出 tensor
4. `decoder_->decode(output_data, shape, meta)` → `vector<RawBuffObject>`
5. `post_.build_result(objs, ...)` → `BuffDetectionResult`

**真正异步推理** (与 auto_aim OV 检测器模式一致):
- `push()`: 创建 `ov::InferRequest`，设置 input tensor，调用 `start_async()`
- `pop()`: 等待异步完成，取回输出 tensor，执行 decoder + postprocess

### 5.5 TensorrtBuffDetector

与 OpenVINO 后端同接口，推理部分替换为 TensorRT:
- 自动缓存 `.engine` 文件
- 支持 FP16 / INT8 量化
- CUDA 预处理 (letterbox + BGR→RGB + normalize)
- 异步推理使用 `cudaStream` + `cudaEvent`

### 5.6 detector_factory.cpp 修改

```cpp
if (type_str == "traditional") {
    return create_traditional_detector(color, config_file);
} else if (type_str == "yolo") {
    auto backend = static_param::get_param<std::string>(config, "Detector", "yolo_backend");
    if (backend == "openvino") {
#ifdef ENABLE_OPENVINO_DETECTOR
        return OpenvinoBuffDetector::from_config(color, config_file);
#else
        throw std::runtime_error("OpenVINO 未编译，请安装 OpenVINO 并重新 cmake");
#endif
    } else if (backend == "tensorrt") {
#ifdef ENABLE_TENSORRT_DETECTOR
        return TensorrtBuffDetector::from_config(color, config_file);
#else
        throw std::runtime_error("TensorRT 未编译，请安装 TensorRT 并重新 cmake");
#endif
    }
}
```

---

## 6. Predictor 详细实现

### 6.1 ObservationBuilder — 检测结果转 3D 观测

```cpp
class ObservationBuilder {
public:
    struct FrameObservation {
        bool pose_valid = false;
        Eigen::Vector3d center_cam, center_world, normal_cam;
        std::array<Eigen::Vector3d, NUM_SLOTS> slot_pos_cam{};
        std::array<Eigen::Vector3d, NUM_SLOTS> slot_vec_cam{};  // center→slot
    };

    FrameObservation build(const BuffDetectionResult& det) const;
};
```

**build() 流程**:
1. 收集 R center + 所有 valid target 的 image points
2. 构造对应的 object points: R=(0,0,0), slot_i=(R*cos(i*72°), R*sin(i*72°), 0)
3. 至少 4 个点时调用 `cv::solvePnP(SOLVEPNP_ITERATIVE)`
4. 提取旋转中心 (translation) 和法向量 (R 矩阵第3列)
5. 计算每个 slot 的 3D 位置和 center→slot 向量

### 6.2 SlotDebouncer — 去抖状态机

```cpp
class SlotDebouncer {
public:
    struct StableSlot {
        bool valid = false;
        bool is_lit = false;
        bool state_changed = false;  // OFF→ON 瞬间
        float confidence = 0.f;
    };

    struct Output {
        std::array<StableSlot, NUM_SLOTS> slots{};
        uint8_t lit_mask = 0;
        int lit_count = 0;
    };

    void reset();
    Output update(const BuffDetectionResult& det, double timestamp);

private:
    // 从 runtime_param 读取 (使用点直接调用):
    // AutoBuff.Predictor.Debounce.on_frames = 3
    // AutoBuff.Predictor.Debounce.off_frames = 4
    // AutoBuff.Predictor.Debounce.missing_timeout = 0.12

    enum class State { OFF, CANDIDATE_ON, ON, CANDIDATE_OFF };
    struct Track { State state = State::OFF; int count = 0; double last_seen = 0; };
    std::array<Track, NUM_SLOTS> tracks_{};
};
```

**状态机转移表**:
| 当前状态 | 输入 | 转移 | 条件 |
|----------|------|------|------|
| OFF | detected & lit | → CANDIDATE_ON | count=1 |
| CANDIDATE_ON | detected & lit | count++ | |
| CANDIDATE_ON | count >= on_frames | → ON | state_changed=true |
| CANDIDATE_ON | !detected | → OFF | count=0 |
| ON | !detected | → CANDIDATE_OFF | count=1 |
| CANDIDATE_OFF | !detected | count++ | |
| CANDIDATE_OFF | count >= off_frames | → OFF | |
| CANDIDATE_OFF | detected & lit | → ON | count=0 |
| 任意 | t - last_seen > timeout | → OFF | 强制超时 |

### 6.3 DirectionEstimator — 方向投票 (★ 集中管理)

```cpp
class DirectionEstimator {
public:
    void reset();
    void feed(double phi_now, double phi_last, double dt);

    RotateDir direction() const;
    int dir_sign() const;  // +1 / -1 / 0

private:
    int votes_ = 0;        // [-20, +20]
    RotateDir dir_ = RotateDir::UNKNOWN;
    static constexpr int CONFIRM_THRESHOLD = 8;
};
```

**所有模型共享同一个 DirectionEstimator 实例**，避免模式切换时方向丢失。
`feed()` 由 `BuffPredictor::predict()` 调用，不在各 model 内部。

### 6.4 ModeManager — 模式判定

```cpp
class ModeManager {
public:
    void reset();
    BuffMode update(aimer::AimMode aim_mode, int debounced_lit_count);
    BuffMode current() const { return mode_; }

private:
    // runtime_param:
    // AutoBuff.Mode.enter_large_active_frames = 3
    // AutoBuff.Mode.exit_large_active_frames = 4
    BuffMode mode_ = BuffMode::UNKNOWN;
    int active_streak_ = 0;
    int inactive_streak_ = 0;
};
```

### 6.5 MotionModelInterface — 运动模型接口

```cpp
namespace autobuff::predictor::models {

class MotionModelInterface {
public:
    virtual ~MotionModelInterface() = default;
    virtual void reset() = 0;

    // 输入: 当前角度测量, 时间戳, 方向 (外部提供)
    virtual void feed(double phi_meas, double timestamp, int dir_sign) = 0;

    virtual MotionEstimate estimate() const = 0;
};

}
```

> **关键设计**: `dir_sign` 由外部 (DirectionEstimator) 提供，模型不自行投票。
> `phi_meas` 是已经做过角度展开的连续相位。

### 6.6 ConstModel — 恒速模型

```cpp
class ConstModel final : public MotionModelInterface {
    void reset() override;
    void feed(double phi_meas, double timestamp, int dir_sign) override;
    MotionEstimate estimate() const override;

private:
    double omega_ = M_PI / 3.0;
    int dir_ = 1;
    bool inited_ = false;
};
```

`estimate()` 返回 `{CONST_OMEGA, dir_ * omega_, ...}`。

### 6.7 SmallEkfModel — 小符 EKF

```cpp
class SmallEkfModel final : public MotionModelInterface {
    // AdaptiveEkf<2, 1>: 状态 [phi, omega], 观测 [phi]
    aimer::filter::AdaptiveEkf<2, 1> ekf_;
    bool inited_ = false;
    double last_t_ = 0;
};
```

**数学定义**:
- 状态: `x = [phi, omega]^T`
- 预测: `phi' = phi + omega*dt`, `omega' = omega`
- 观测: `z = phi_meas`
- 过程噪声: Q 从 `AutoBuff.Predictor.SmallEKF.q_phi/q_omega` 读取
- 观测噪声: R 从 `AutoBuff.Predictor.SmallEKF.r_phi` 读取
- omega 先验: 软约束到 `dir * π/3`，不做硬固定

**Predict/Measure 仿函数** (Ceres Jet 自动微分):
```cpp
struct SmallEkfPredict {
    double dt;
    template <typename T>
    void operator()(const T x_in[2], T x_out[2]) const {
        x_out[0] = x_in[0] + T(dt) * x_in[1];
        x_out[1] = x_in[1];
    }
};

struct SmallEkfMeasure {
    template <typename T>
    void operator()(const T x[2], T y[1]) const {
        y[0] = x[0];
    }
};
```

**异常处理**:
- `dt < 1e-4 || dt > 0.2` → 重置 EKF
- `|omega_ekf - dir*π/3| > π/6` → 重置 EKF (发散保护)

### 6.8 LargeLsmModel — 大符最小二乘拟合

```cpp
class LargeLsmModel final : public MotionModelInterface {
    struct Sample { double t_rel; double phi; };
    std::deque<Sample> samples_;
    LargeSineParam param_;
    bool active_ = false;
    double start_time_ = 0;
    double phi_unwrap_ = 0;    // 连续相位 (展开后)
    double last_phi_raw_ = 0;
    bool has_last_ = false;
};
```

**拟合模型**:
```
phi_pred(t) = dir * (-(a/w)*cos(w*(t+tau)) + b*t) + phi0
其中 b = 2.090 - a
```

**Ceres 代价函数**:
```cpp
struct LargePhiResidual {
    double t_, y_;
    int dir_;

    template <typename T>
    bool operator()(const T* const p, T* residual) const {
        // p = [a, w, tau, phi0]
        const T a = p[0], w = p[1], tau = p[2], phi0 = p[3];
        const T b = T(2.090) - a;
        const T pred = T(dir_) * (-(a/w) * ceres::cos(w * (T(t_) + tau)) + b * T(t_)) + phi0;
        residual[0] = pred - T(y_);
        return true;
    }
};
```

**参数边界**:
```cpp
problem.SetParameterLowerBound(p, 0, 0.780);  // a_min
problem.SetParameterUpperBound(p, 0, 1.045);  // a_max
problem.SetParameterLowerBound(p, 1, 1.884);  // w_min
problem.SetParameterUpperBound(p, 1, 2.000);  // w_max
problem.SetParameterLowerBound(p, 2, -0.5);   // tau_min
problem.SetParameterUpperBound(p, 2, 0.5);    // tau_max
```

**feed() 流程**:
1. 若刚进入 LARGE_ACTIVE → reset: `start_time = now`, 清空 samples
2. 相位展开: `phi_unwrap_ += normalize(phi_raw - last_phi_raw_)`
3. 推入样本: `samples_.push_back({t - start_time_, phi_unwrap_})`
4. 滑窗裁剪: 保留最近 `window_sec` 秒 (从 `AutoBuff.Predictor.LargeLSM.window_sec` 读取)
5. 满足条件时求解:
   - `samples_.size() >= min_samples` (35)
   - `t_span >= min_span_sec` (0.6s)
6. Ceres 求解:
   - `HuberLoss(0.1)` 鲁棒核
   - `DENSE_QR`, max 40 iterations
   - warm start (从上次 param_ 初始化)
7. 结果评估:
   - `IsSolutionUsable() == false` → `param_.valid = false`
   - `residual_rms > residual_accept` (0.18) → `param_.valid = false`
   - 否则 → `param_.valid = true`

**相位展开** (★ 关键算法, 处理 72° 扇叶切换):
```cpp
// normalize_angle: 归一化到 (-π, π]
double dphi = normalize_angle(phi_raw - last_phi_raw_);
phi_unwrap_ += dphi;
last_phi_raw_ = phi_raw;
```
这天然处理了扇叶切换：因为 `phi_raw` 是相对 R 标的角度，切换扇叶时角度跳变约 72°，
`normalize_angle` 会把 >180° 的跳变折叠，而正常的 72° 跳变 (<180°) 会被正确累加。

### 6.9 BuffPredictor — 主预测器 (★ 重写)

```cpp
class BuffPredictor {
public:
    BuffPredictor();
    void reset();
    BuffSnapshot predict(const BuffDetectionResult& det);

private:
    ObservationBuilder obs_builder_;
    SlotDebouncer debouncer_;
    DirectionEstimator dir_estimator_;  // ★ 集中管理
    ModeManager mode_mgr_;

    models::ConstModel const_model_;
    models::SmallEkfModel small_model_;
    models::LargeLsmModel large_model_;

    int last_track_slot_ = -1;
    double last_track_phi_ = 0.0;
    double last_timestamp_ = 0.0;
    bool has_last_track_ = false;

    int choose_track_slot(const SlotDebouncer::Output& debounced,
                          const BuffDetectionResult& det) const;
    void build_ccw_rank(BuffSnapshot& snap) const;
};
```

**predict() 固定流程**:
```cpp
BuffSnapshot BuffPredictor::predict(const BuffDetectionResult& det) {
    BuffSnapshot snap;
    snap.frame_id = det.frame_id;
    snap.timestamp = det.timestamp;
    snap.self_state = det.robot_state;

    // ① 3D 观测 (PnP)
    auto obs = obs_builder_.build(det);

    // ② 去抖
    auto debounced = debouncer_.update(det, det.timestamp);
    snap.lit_mask = debounced.lit_mask;
    snap.lit_count = debounced.lit_count;

    // ③ 模式判定
    snap.mode = mode_mgr_.update(det.robot_state.aim_mode, debounced.lit_count);
    if (snap.mode == BuffMode::UNKNOWN) {
        reset();  // 非能量机关模式 → 全部重置
        snap.valid = false;
        return snap;
    }

    // ④ 选择跟踪槽位 (用于方向/拟合)
    int track = choose_track_slot(debounced, det);
    double phi_now = (track >= 0) ? det.targets[track].angle : 0.0;

    // ⑤ 方向投票 (★ 集中, 所有模型共享)
    if (track >= 0 && has_last_track_) {
        double dt = det.timestamp - last_timestamp_;
        dir_estimator_.feed(phi_now, last_track_phi_, dt);
    }
    snap.direction = dir_estimator_.direction();
    int dir = dir_estimator_.dir_sign();
    if (dir == 0) dir = 1;  // 默认 CCW

    // ⑥ 模型分发
    if (track >= 0) {
        switch (snap.mode) {
            case BuffMode::SMALL_ACTIVE:
                small_model_.feed(phi_now, det.timestamp, dir);
                snap.motion = small_model_.estimate();
                break;
            case BuffMode::LARGE_INACTIVE:
                const_model_.feed(phi_now, det.timestamp, dir);
                snap.motion = const_model_.estimate();
                break;
            case BuffMode::LARGE_ACTIVE:
                large_model_.feed(phi_now, det.timestamp, dir);
                snap.motion = large_model_.estimate();
                // 拟合失败 → 降级恒速
                if (snap.motion.model != SpeedModel::LARGE_SINE_LSM
                    || !snap.motion.large.valid) {
                    const_model_.feed(phi_now, det.timestamp, dir);
                    snap.motion = const_model_.estimate();
                }
                break;
            default: break;
        }
    }

    // ⑦ 填充 3D 信息
    if (obs.pose_valid) {
        snap.center_cam = obs.center_cam;
        snap.center_world = obs.center_world;
        snap.normal_cam = obs.normal_cam;
        for (int i = 0; i < NUM_SLOTS; ++i) {
            if (!debounced.slots[i].valid) continue;
            snap.slots[i].valid = true;
            snap.slots[i].is_lit = debounced.slots[i].is_lit;
            snap.slots[i].confidence = det.targets[i].confidence;
            snap.slots[i].center_px = det.targets[i].center;
            snap.slots[i].angle = det.targets[i].angle;
            snap.slots[i].pos_cam = obs.slot_pos_cam[i];
            snap.slots[i].pos_world = aimer::tf::cam_to_world(
                obs.slot_pos_cam[i], det.robot_state.q_imu);
            snap.slots[i].vec_cam = obs.slot_vec_cam[i];
        }
        snap.valid = true;
    }

    // ⑧ CCW 排序 (双车协同用)
    build_ccw_rank(snap);

    // 更新跟踪状态
    if (track >= 0) {
        last_track_slot_ = track;
        last_track_phi_ = phi_now;
        has_last_track_ = true;
    }
    last_timestamp_ = det.timestamp;

    return snap;
}
```

### 6.10 CCW 排序算法 (★ 双车协同核心)

```cpp
void BuffPredictor::build_ccw_rank(BuffSnapshot& snap) const {
    // 收集 lit slots 的角度
    struct LitSlot { int slot_id; double angle; };
    std::vector<LitSlot> lits;
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (snap.slot_lit(i)) {
            lits.push_back({i, snap.slots[i].angle});
        }
    }

    if (lits.empty()) {
        snap.ranked_count = 0;
        return;
    }

    // 按逆时针排序 (角度递增 = CCW)
    // 若旋转方向是 CW，则 "逆时针第1个" 是角度最大的 (先被转到的)
    // 若旋转方向是 CCW，则 "逆时针第1个" 是角度最小的 (先被转到的)
    if (snap.direction == RotateDir::CW) {
        // CW 旋转: 角度递减方向是运动方向
        // "逆时针数第1个" = 运动方向的反方向第1个 = 角度最大的
        std::sort(lits.begin(), lits.end(),
                  [](auto& a, auto& b) { return a.angle > b.angle; });
    } else {
        // CCW 旋转: 角度递增方向是运动方向
        // "逆时针数第1个" = 角度最小的
        std::sort(lits.begin(), lits.end(),
                  [](auto& a, auto& b) { return a.angle < b.angle; });
    }

    snap.ranked_count = static_cast<int>(lits.size());
    for (int i = 0; i < snap.ranked_count && i < NUM_SLOTS; ++i) {
        snap.ccw_lit_rank[i] = lits[i].slot_id;
    }

    // recommended_slot 默认取 rank[0]
    snap.recommended_slot = snap.ccw_lit_rank[0];
}
```

---

## 7. FireControl 详细实现

### 7.1 TargetRanker — 候选构建与评分

```cpp
class TargetRanker {
public:
    std::vector<SlotAimCandidate> build(
        const predictor::BuffSnapshot& snap,
        const ::fire_control::LatencyInfo& latency,
        const ::fire_control::GimbalState& gimbal) const;
};
```

**build() 流程**:
1. 遍历 lit slots (无 lit 则回退 valid slots)
2. 对每个候选:
   ```cpp
   double dt = latency.prediction_latency();
   Eigen::Vector3d p = snap.predict_slot_world(slot, dt);
   auto aim = trajectory::solve(p, bullet_speed);
   double err = hypot(normalize(aim.yaw - gimbal.yaw),
                      aim.pitch - gimbal.pitch);
   double score = -err * w_error + conf * w_conf;
   ```
3. 填充 `ccw_rank` 从 `snap.ccw_lit_rank` 反查
4. 按 `score` 降序排列

### 7.2 CoopPolicy — 双车协同策略

```cpp
class CoopPolicy {
public:
    int select(
        const predictor::BuffSnapshot& snap,
        const std::vector<SlotAimCandidate>& cands) const;
};
```

**select() 逻辑**:
```cpp
// 从 runtime_param 读取 (每次调用直接读，不缓存!)
auto role_str = runtime_param::get_param<std::string>("AutoBuff.FireControl.coop_role");
auto coop_only_large = runtime_param::get_param<bool>("AutoBuff.FireControl.coop_only_large_active");

CoopRole role = parse_coop_role(role_str);  // DISABLED/CCW_FIRST/CCW_SECOND

// 不协同 → 返回最高 score 的候选
if (role == CoopRole::DISABLED) return best_score_index;

// 非大符激活 + 仅大符协同 → 返回最高 score
if (coop_only_large && snap.mode != BuffMode::LARGE_ACTIVE) return best_score_index;

// 协同选择
int target_rank = (role == CoopRole::CCW_FIRST) ? 0 : 1;

// 在候选中找 ccw_rank == target_rank 的
for (auto& c : cands) {
    if (c.ccw_rank == target_rank && c.ballistic_valid) return c.slot_id;
}
// 指定 rank 无弹道可解 → 回退最高 score
return best_score_index;
```

### 7.3 FireController — 火控主循环 (★ 重写)

```cpp
class FireController {
public:
    void reset();
    ::fire_control::FireCommand control(
        const predictor::BuffSnapshot& snapshot,
        double current_time,
        const ::fire_control::LatencyInfo& latency);

private:
    TargetRanker ranker_;
    CoopPolicy coop_;
    ::fire_control::GimbalState gimbal_;
    double last_time_ = 0;
    int lost_count_ = 0;
};
```

**control() 流程**:
1. 更新 `gimbal_.update(snapshot.self_state.q_imu, dt)`
2. snapshot 无效 → `no_target_command()`
3. `cands = ranker_.build(snapshot, latency, gimbal_)`
4. `chosen = coop_.select(snapshot, cands)` → 获取最终目标 slot
5. 找到 chosen 在 cands 中的 `SlotAimCandidate`
6. 开火判断 (参数直接从 runtime_param 读取):
   ```cpp
   double threshold = runtime_param::get_param<double>("AutoBuff.FireControl.fire_threshold");
   double min_conf = runtime_param::get_param<double>("AutoBuff.FireControl.min_confidence");

   bool fire = snapshot.self_state.allow_fire
            && chosen.confidence >= min_conf
            && chosen.tracking_error <= threshold
            && chosen.ballistic_valid;
   ```
7. 输出 `FireCommand`

### 7.4 fire_control_node.cpp 修改

保持现有结构:
- 500Hz 固定频率循环
- `LatencyEstimator` 更新 `predict_to_send`
- 模式切换时 `controller.reset()`
- `finalize_latency` 迭代弹道飞行时间

---

## 8. 配置文件

### 8.1 `config/buff.toml` — 新增 YOLO 段

```toml
[Detector]
    type = "yolo"              # "traditional" 或 "yolo"
    debug = false

[Detector.Yolo]
    backend = "openvino"       # "openvino" 或 "tensorrt"
    model_path = "buff_25_best.onnx"
    input_size = 640
    conf_threshold = 0.45
    nms_threshold = 0.45

# 传统检测器配置 (保留不变)
[TraditionalDetector]
    # ... 原有参数保持不变
```

### 8.2 `config/aimer.toml` — 新增 AutoBuff 段

```toml
[AutoBuff.Mode]
    enter_large_active_frames = 3
    exit_large_active_frames = 4

[AutoBuff.Predictor.Debounce]
    on_frames = 3
    off_frames = 4
    missing_timeout = 0.12

[AutoBuff.Predictor.SmallEKF]
    q_phi = 0.0002
    q_omega = 0.005
    r_phi = 0.004

[AutoBuff.Predictor.LargeLSM]
    window_sec = 2.2
    min_samples = 35
    min_span_sec = 0.6
    huber_delta = 0.1
    residual_accept = 0.18

[AutoBuff.FireControl]
    fire_threshold = 0.02
    min_confidence = 0.30
    coop_role = "DISABLED"     # DISABLED / CCW_FIRST / CCW_SECOND
    coop_only_large_active = true

[AutoBuff.FireControl.Latency]
    send_to_control = 0.003
    control_to_fire = 0.020
```

---

## 9. 容错与降级策略

| 场景 | 处理 |
|------|------|
| R 标检测失败 | 用 kpt[5] 推导的估计值; 若无 fan 检测则 status=NONE |
| PnP 失败 (<4 点) | `snap.valid=false`, fire_control 输出 no_target |
| EKF 发散 | 自动重置 (omega 偏离先验过多) |
| 大符拟合失败 | 降级 ConstModel (ω=π/3), 不中断火控 |
| 大符拟合 RMS 过大 | 视为无效, 降级 ConstModel |
| 模式切换 (LARGE→SMALL) | 清空 large samples, 重置 large_model |
| 队列拥堵 (async) | 丢弃最旧帧, 保持低延迟 |
| 协同指定 rank 无候选 | 回退 best score |
| 连续 50 帧无效 | reset fire_controller |

---

## 10. 日志与可观测性

### 10.1 Dashboard 键 (全部新增)
```
buff_detector.backend          int     当前检测器后端
buff_detector.latency_ms       float   检测延迟
buff_detector.lit_count        int     检测到亮扇叶数
buff_predictor.mode            int     BuffMode 枚举值
buff_predictor.model           int     SpeedModel 枚举值
buff_predictor.omega           double  当前角速度
buff_predictor.fit_rms         double  大符拟合 RMS (仅 LARGE_SINE)
buff_predictor.direction       int     旋转方向 (-1/0/+1)
buff_fire.selected_slot        int     选中的槽位
buff_fire.selected_rank        int     CCW 排序位置
buff_fire.tracking_error       double  跟踪误差 (rad)
buff_fire.fire_now             int     是否开火 (0/1)
```

### 10.2 日志点 (debug::print)
1. 模式切换: `"Mode: {} -> {}"` (每次转变)
2. 方向确定: `"Direction locked: {}"` (首次确定时)
3. 大符拟合: `"LSM fit: a={:.3f} w={:.3f} tau={:.3f} rms={:.4f} N={}"` (每次拟合)
4. 大符拟合失败: `"LSM fit FAILED: {}"` (原因)
5. 协同选择: `"Coop: role={} rank={} slot={}"` (每次选择)
6. EKF 重置: `"SmallEKF reset: omega deviated"` (发散重置时)

---

## 11. 分阶段提交顺序

| 阶段 | 内容 | 验收标准 |
|------|------|----------|
| PR-1 | 类型扩展 + 目录骨架 + CMake | 编译通过, 空实现 |
| PR-2 | Sp25Decoder + Preprocess + Postprocess | 离线图片可解码 |
| PR-3 | OpenVINO 后端 | 实时推理, dashboard 有输出 |
| PR-4 | TensorRT 后端 | 同 OV, GPU 推理 |
| PR-5 | ObservationBuilder + SlotDebouncer + DirectionEstimator | 3D 观测稳定, 去抖有效 |
| PR-6 | ModeManager + ConstModel + SmallEkfModel | 小符跟踪准确 |
| PR-7 | LargeLsmModel | 大符拟合收敛 |
| PR-8 | TargetRanker + CoopPolicy + FireController 重写 | 双车协同可验证 |
| PR-9 | 参数整定 + 测试 | 回放/实机验证 |

---

## 12. 与 Codex 原方案的关键差异

| 问题 | Codex 原方案 | 本方案修正 |
|------|------------|-----------|
| R 标检测 | 假设模型有 R 类输出 | 从 kpt[5] 推导 + 传统 CV 精修 |
| Lit 判定 | 假设模型有 lit_score | 被检测到 = lit (模型只检测亮的) |
| 方向投票 | 分散在 3 个 model 中 | 集中到 DirectionEstimator |
| CCW 排序 | 未定义算法 | 完整排序算法 |
| FireControl 参数 | 缓存在 cfg_ struct | 使用点直接 runtime_param 读取 |
| Observation 层 | 4 个独立类 | 合并为 3 个 (builder/debouncer/方向估计) |
| RawClassId | FAN/R/UNKNOWN 枚举 | 删除 (单类模型不需要) |
| lit_score/lit_thresh | PostprocessConfig 中 | 删除 (检测到=lit) |
| 角度展开 | 仅 LargeLsmModel 有 | 在 BuffPredictor 层统一处理 |
| predict_slot_world | 签名存在但无实现 | 通过 MotionEstimate.delta_theta 统一实现 |
