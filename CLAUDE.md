# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RMCV is a C++17 computer vision project for robotic applications, specifically designed for autonomous targeting and detection systems. The project uses a CMake-based build system and integrates hardware control, computer vision processing, and parameter management.

## Build System

### Dependencies
- **CMake 3.16+** with C++17 standard
- **OpenCV** - Computer vision library
- **fmt** - String formatting library
- **Eigen3** - Linear algebra library
- **tomlplusplus** - TOML configuration parsing
- **Python 3.6+** with development headers
- **pybind11** - Python bindings
- **MVS (Machine Vision SDK)** - HIK camera control library

### Build Commands

```bash
# Configure and build (Release mode with optimizations)
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Build with debug information
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j$(nproc)

# Build minimal size
cmake -DCMAKE_BUILD_TYPE=MinSizeRel ..
make -j$(nproc)

# Run tests
./test_param

# Run main application
./RMCV2026
```

### Build Features
- **ccache integration** for faster rebuilds
- **Link-time optimization (LTO)** for performance
- **Native architecture optimizations** (`-march=native`)
- **Strict compiler warnings** with `-Werror=return-type`

## Code Architecture

### Core Modules

1. **UMT (UltraMultiThread)** - Custom thread-safe messaging and object management
   - `Message.hpp` - Thread-safe message passing with Publisher/Subscriber pattern
   - `ObjManager.hpp` - Object lifecycle management for class types
   - `BasicObjManager.hpp` - Object management for basic types (int, bool, float, structs)
   - Located in `umt/` directory

2. **Hardware Layer** - Hardware abstraction and control
   - `hardware/` - Hardware interface library
   - `hardware/hik_cam/` - HIK camera control (MVS SDK)
   - `hardware/serial/` - Serial communication protocols
   - Supports both UART and USB bulk transfer protocols

3. **Plugin System** - Core utilities and parameter management
   - `plugin/debug/` - Logging system with markdown output
   - `plugin/param/` - Configuration management (static and runtime)
   - TOML-based configuration files in `config/`

4. **Auto-Aim System** - Computer vision for autonomous targeting
   - `aimer/auto_aim/detector/` - Target detection algorithms
   - `aimer/common/` - Mathematical utilities and transformations

### Parameter Management

The project uses a dual-parameter system:

- **Static Parameters** (`static_config.hpp`) - Load once from TOML at startup
- **Runtime Parameters** (`runtime_parameter.hpp`) - Dynamic parameter updates

Configuration files are located in `config/` with `.toml` extension:
- `hardware.toml` - Camera and serial port settings
- `detector.toml` - Detection algorithm parameters
- `aimer.toml` - Auto-aim system configuration
- `test.toml` - Testing parameters

### Threading Model

- Main application runs parameter management in separate thread
- UMT provides thread-safe message passing between components
- Camera capture and processing run in dedicated threads

## Development Workflow

### Code Style

The project enforces strict code formatting using:
- **.clang-format** - 100-character line limit, 4-space indentation
- **.clang-tidy** - Modern C++ practices, bug-prone pattern detection

Naming conventions (enforced by clang-tidy):
- Classes/Structs: `CamelCase`
- Functions/Variables: `lower_case`
- Namespaces: `lower_case`
- Macros/Constants: `UPPER_CASE`

Language conventions:
- 代码注释使用简单的中文
- Log输出和debug信息使用英文

### Testing

Test executable: `test_param` - Validates parameter loading and runtime configuration

```bash
# Run parameter tests
./build/test_param
```

### Directory Structure

```
RMCV/
├── aimer/           # Auto-aim computer vision modules
├── config/          # TOML configuration files
├── hardware/        # Hardware abstraction layer
├── plugin/          # Core utilities and parameter system
├── test/            # Test programs
├── umt/             # Custom threading/messaging library
├── main.cpp         # Application entry point
└── CMakeLists.txt   # Main build configuration
```

## Important Notes

- **Configuration Paths**: Asset, config, and log directories are defined at compile time via `ASSET_DIR`, `CONFIG_DIR`, and `LOG_DIR` macros
- **Camera Integration**: Requires HIK MVS SDK installation and proper camera configuration files (`.mfs`)
- **Serial Communication**:
  - Supports both UART (/dev/ttyUSB*) and USB Bulk protocols
  - UART requires standard Linux serial device permissions
  - USB Bulk requires libusb-1.0 and proper device permissions
  - Uses thread-safe UMT object management for data sharing
- **UMT Object Management**:
  - `ObjManager<T>` for class types only (inherits from T)
  - `BasicObjManager<T>` for basic types and structs (wraps T)
  - Message system for thread-safe publisher/subscriber communication
- **Logging**: Uses custom markdown-based logging system with colorized console output
- **Error Handling**: Extensive use of `std::optional` and variant types for safe parameter handling

## Common Development Tasks

### Adding New Hardware Components
1. Create subdirectory in `hardware/`
2. Implement interface following existing patterns
3. Add to `hardware/CMakeLists.txt`
4. Update configuration TOML files

### Parameter System Usage
```cpp
// Static parameters (load once)
auto param = static_param::parse_file("config.toml");
auto value = static_param::get_param<std::string>(param, "section", "key");

// Runtime parameters (dynamic updates)
runtime_param::parameter_run("config.toml");
auto dynamic_value = runtime_param::get_param<std::string>("section.key");
```

### UMT Object Management
```cpp
// BasicObjManager - 用于基本类型和结构体
#include "umt/BasicObjManager.hpp"

// 创建或查找基本类型对象
auto bool_obj = umt::BasicObjManager<bool>::find_or_create("enable_flag", true);
auto float_obj = umt::BasicObjManager<float>::find_or_create("threshold", 0.5f);

// 访问和修改数据
bool_obj->get() = false;  // 设置值
bool enabled = bool_obj->get();  // 获取值

// 对于自定义结构体
struct MyData {
    int id;
    float value;
};
auto data_obj = umt::BasicObjManager<MyData>::find_or_create("my_data");
data_obj->get().id = 42;
data_obj->get().value = 3.14f;

// Message系统 - 用于线程间通信
#include "umt/Message.hpp"

// Publisher发布消息
umt::Publisher<MyData> publisher("data_channel");
MyData msg{42, 3.14f};
publisher.push(msg);

// Subscriber订阅消息
umt::Subscriber<MyData> subscriber("data_channel");
try {
    MyData received = subscriber.pop();  // 阻塞等待消息
    MyData received_timeout = subscriber.pop_for(1000);  // 1秒超时
} catch (const umt::MessageError_Timeout&) {
    // 处理超时
}
```

### Debug Logging
```cpp
debug::init_md_file("log.log");  // Initialize markdown logger
debug::print("info", "module", "message: {}", data);
```

### Serial Communication Module
#### 串口模块架构
串口模块位于 `hardware/serial/` 目录，采用分层架构设计：

**核心组件：**
- `SerialNode` - 串口节点管理器，负责启动收发线程
- `TransceiverManager` - 收发管理器，封装底层协议
- `FixedPacket` - 定长数据包，支持帧头帧尾校验
- `Protocol Interface` - 协议接口，支持UART和USB Bulk传输

**协议支持：**
- `UartProtocol` - 标准UART串口通信
- `UsbBulkProtocol` - USB Bulk传输协议（需要libusb-1.0）

#### 数据类型定义
```cpp
// 视觉数据结构
struct VisionData_t {
    uint8_t cmd_id;      // 命令ID
    float yaw;           // 偏航角
    float pitch;         // 俯仰角
    float distance;      // 距离
    uint8_t target_id;   // 目标ID
    uint8_t is_found;    // 是否发现目标
};

// 接收数据结构
struct SerialReceiveData {
    uint8_t cmd_id;      // 命令ID
    float yaw;           // 偏航角
    float pitch;         // 俯仰角
    float distance;      // 距离
    uint64_t timestamp;  // 时间戳
};
```

#### 串口使用方法
```cpp
#include "hardware/serial/serial_node.hpp"

// 启动串口通信（自动创建收发线程）
serial::start_serial_communication("/dev/ttyUSB0", 115200);

// 通过UMT共享数据
// 发送视觉数据
auto vision_data = umt::BasicObjManager<VisionData_t>::find_or_create("vision_transmit");
vision_data->get() = my_vision_data;

// 接收数据队列
auto recv_queue = umt::BasicObjManager<std::queue<SerialReceiveData>>::find_or_create("receive_queue");
```

#### FixedPacket 数据包操作
```cpp
// 创建16字节数据包
FixedPacket<16> packet;

// 装载数据到指定位置（字节偏移）
packet.load_data(float_value, 1);    // 在偏移1处装载float
packet.load_data(uint8_value, 5);    // 在偏移5处装载uint8_t

// 提取数据
float value;
if (packet.unload_data(value, 1)) {
    // 成功提取数据
}

// 数据包格式: [HEAD_BYTE][DATA...][CHECK_BYTE][TAIL_BYTE]
// HEAD_BYTE = 0xff, TAIL_BYTE = 0x0d
```

#### 线程模型
- **发送线程** - 从UMT获取视觉数据，转换为数据包并发送
- **接收线程** - 接收数据包，解析后存入共享队列
- **共享实例** - 收发线程共享同一个串口/协议实例

#### 配置文件支持
串口参数通过TOML配置文件管理：
```toml
[serial]
port_path = "/dev/ttyUSB0"
baud_rate = 115200
timeout_ms = 100
```

## 参考项目: rm.cv.fans (SJTU Lmtd)

来自上海交通大学 RoboMaster 战队的视觉系统，以下是值得借鉴的设计模式和最佳实践：

### 设计模式


#### 3. 自动微分EKF
使用Ceres库的Jet类型实现自动微分，无需手动计算雅可比矩阵：
```cpp
template<int N_X, int N_Y>
class AdaptiveEkf {
    template<class PredictFunc>
    PredictResult predict(PredictFunc&& func) {
        ceres::Jet<double, N_X> x_jet[N_X];
        func(x_jet, result_jet);  // 自动计算导数
        // 从jet提取雅可比矩阵
    }
};
```

### 参数系统增强

#### 运行时热重载
```cpp
// 参数持续监控文件变化，自动重载
base::parameter_run("config.toml");  // 异步线程持续更新

// 类型安全的点分隔访问
auto value = base::get_param<double>("launching-mechanism.bullet.resistance-k");
```

#### TOML层级配置
```toml
[launching-mechanism]
camera-to-barrel-x = 0.0

    [launching-mechanism.bullet]
    radius = 0.0085
    mass = 0.041
```

### 通用工具类

#### FPS统计插件 (待实现)
```cpp
// plugin/stats/fps_stats.hpp
struct FpsStats {
    std::string module;
    int count = 0;
    int secondary = 0;
    std::string secondary_label;
    SteadyClock::time_point last_print;

    void tick(float latency = 0, bool secondary_hit = false);
    void print_if_needed();
};

// 使用
FpsStats stats("DetectorNode", "detected");
stats.tick(latency, !armors.empty());
```

#### 数学工具
```cpp
// 球坐标结构
struct YpdCoord {
    double yaw, pitch, dis;
};

// 常用工具函数
template<typename T> T sq(const T& x) { return x * x; }
inline double get_ratio(double x, double y) { return (x < y) ? x / y : y / x; }
```

### 架构参考

#### 生产者-消费者管道
```
Camera Thread → Detector Thread → Predictor Thread → Controller Thread
     ↓                ↓                 ↓                  ↓
  SyncFrame    DetectionResult     PredictResult       RobotCmd
```

#### 无状态组件设计
每个模块接收共享状态的引用，而不是缓存状态：
```cpp
class EnemyModel {
    EnemyModel(CoordConverter* converter, EnemyState* state);
    // 方法始终使用当前的converter和state
};
```

### 可借鉴的改进方向

| 功能 | 当前RMCV | rm.cv.fans | 改进建议 |
|------|----------|------------|----------|
| 参数加载 | 启动时加载一次 | 运行时热重载 | 添加文件监控线程 |
| 滤波器 | 简单工厂函数 | Factory + Builder | 采用Builder模式 |
| 类型约束 | SFINAE | C++20 Concepts | 升级到Concepts |
| EKF雅可比 | 手动计算 | Ceres自动微分 | 考虑引入Ceres |
| 统计信息 | 各模块重复代码 | 通用FpsStats | 创建plugin/stats |
| 网页调参 | 无 | webview_info树形结构 | 实现Web UI |

### 网页调参系统 (待实现)

rm.cv.fans使用树形结构管理Web UI数据，通过UMT ObjManager在C++和Python间共享：

#### 数据结构
```cpp
// 树形页面结构
// root (Page)
// ├── 自瞄-识别器 (Group)
// │   ├── 帧率 (Entry) = "30"
// │   └── 延迟 (Entry) = "5.2ms"
// └── 自瞄-预测器 (Group)
//     └── 状态 (Entry) = "tracking"

// 创建或获取页面
auto page = umt::ObjManager<webview_info::Page>::find_or_create("root");

// 添加数据 (树形访问)
page->sub("自瞄-识别器").sub("帧率").get() = fmt::format("{}", fps);
page->sub("自瞄-识别器").sub("延迟").get() = fmt::format("{:.1f}ms", latency);
```

#### 图像流推送
```cpp
// 发布图像供Web显示
umt::Publisher<cv::Mat> img_pub("auto_aim.detector.preview");
img_pub.push(debug_image);

// CheckBox控制是否推送
auto checkbox = umt::ObjManager<CheckBox>::find_or_create("auto_aim.detector");
if (checkbox->checked) {
    img_pub.push(debug_image);
}
```

#### Python Web后端集成
```python
# Python通过pybind11访问C++数据
import ObjManager_Page as page_mgr

# 获取页面数据
page = page_mgr.find("root")
detector_fps = page.sub("自瞄-识别器").sub("帧率").get()

# Flask/FastAPI提供HTTP接口
@app.get("/api/status")
def get_status():
    return {"detector_fps": detector_fps}
```

### Python绑定 (pybind11)

#### 嵌入式模块导出
```cpp
// 导出函数到Python
PYBIND11_EMBEDDED_MODULE(auto_aim_detector, m) {
    m.def("background_detector_run", background_detector_run,
          py::arg("model_path"));
}

// 导出UMT ObjManager
UMT_EXPORT_OBJMANAGER_ALIAS(MyData, MyData, c) {
    c.def(pybind11::init<>());
    c.def_readwrite("value", &MyData::value);
}

// 导出UMT Message
UMT_EXPORT_MESSAGE_ALIAS(MyData, MyData, c) {
    c.def_readwrite("value", &MyData::value);
}
```

#### Python调用C++模块
```python
# 启动参数热重载线程
import base_param
base_param.background_parameter_run("config.toml")

# 启动检测器线程
import auto_aim_detector
auto_aim_detector.background_detector_run("model.onnx")

# 订阅检测结果
import Message_DetectionResult as msg
sub = msg.Subscriber("detections")
result = sub.pop_for(1000)  # 1秒超时
```

### 完整线程架构
```
┌─────────────────────────────────────────────────────────────┐
│                    Python 主程序                            │
│  - Flask/FastAPI Web服务                                    │
│  - 参数修改接口                                              │
│  - 图像流WebSocket                                          │
└─────────────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        ▼                 ▼                 ▼
   参数热重载        Hardware线程       Detector线程
   (1秒/次)         (相机+串口)        (目标检测)
        │                 │                 │
        │            SyncFrame      DetectionResult
        │                 │                 │
        └────────UMT Message Channel────────┘
                          │
                          ▼
                  ObjManager<Page>
                   (Web UI数据)
```

### 参考项目: CVRM2021 (SJTU)

CVRM2021提供了更完整的Python端Web UI实现，可作为网页调参的参考：

#### Flask Web服务器架构
```python
# script/app.py
from flask import Flask, Response, render_template, request
import bridge

app = Flask(__name__)

@app.route('/')
def index():
    return render_template("index.html",
                           video_names=bridge.get_cvmat_names(),
                           params_info=bridge.get_range_params_info(),
                           buttons_info=bridge.get_buttons_info(),
                           checkboxes_info=bridge.get_checkboxes_info())

@app.route('/video_feed/<name>')
def video_feed(name):
    return Response(bridge.get_cvmat_jpegcode(name),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/setting/<param_name>', methods=["GET"])
def setting(param_name):
    value = float(request.args["current_value"])
    bridge.get_range_param(param_name).current_value = value
    return str(value)
```

#### MJPEG视频流实现
```python
# script/bridge.py
import Message_cvMat
import cv2

def get_cvmat_jpegcode(name):
    """生成MJPEG流"""
    sub = Message_cvMat.Subscriber(name, 1)  # fifo_size=1只保留最新帧
    while True:
        mat = sub.pop()
        jpeg = cv2.imencode(".jpeg", mat.get_nparray())[1].tobytes()
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n\r\n'
```

#### Web UI工具类 (C++导出)
```cpp
// common/common.cpp
struct RangeParam {
    double current_value = 0;
    double min_value = 0;
    double max_value = 255;
    double step_value = 1;
};

class Button {
public:
    bool is_press_once();
    void set_press_once();
private:
    bool is_press = false;
};

struct CheckBox {
    bool checked = false;
};

// cv::Mat到numpy转换
UMT_EXPORT_MESSAGE_ALIAS(cvMat, cv::Mat, c) {
    c.def("get_nparray", cvMat2npArray);
}

UMT_EXPORT_OBJMANAGER_ALIAS(RangeParam, RangeParam, c) {
    c.def_readwrite("current_value", &RangeParam::current_value);
    c.def_readwrite("min_value", &RangeParam::min_value);
    c.def_readwrite("max_value", &RangeParam::max_value);
    c.def_readwrite("step_value", &RangeParam::step_value);
}
```

#### HTML调参界面
```html
<!-- templates/index.html -->
<h2>参数设置</h2>
{% for name, param in params_info %}
<li>
    <span>{{ name }}</span>
    <input type="range" min="{{ param.min_value }}" max="{{ param.max_value }}"
           step="{{ param.step_value }}" value="{{ param.current_value }}"
           onchange="range_onchange('{{ name }}')">
</li>
{% endfor %}

<h2>实时视频</h2>
{% for name in video_names %}
<li><a href="/video/{{name}}">{{ name }}</a></li>
{% endfor %}

<script>
function range_onchange(name) {
    var value = document.getElementById("range_" + name).value;
    fetch("/setting/" + name + "?current_value=" + value);
}
</script>
```

#### Python启动脚本模式
```python
# 启动命令: ./RMCV --script sensors.py autoaim.py app.py

# sensors.py
import SensorsIO
SensorsIO.background_sensors_io_auto_restart(
    camera_name="main-cam",
    camera_cfg="config/camera.mfs"
)

# autoaim.py
import AutoAim
AutoAim.background_detection_run("models/armor.onnx")
AutoAim.background_predict_run()

# app.py (最后启动Flask)
app.run(host="0.0.0.0", port=3000, threaded=True)
```

#### 数据流架构
```
[SensorsIO线程] → SensorsData消息
        ↓
[AutoAim检测线程] → detections消息 (cv::Mat画框图)
        ↓                    ↓
[AutoAim预测线程]      [Web Server]
        ↓                    ↓
    robot_cmd消息      MJPEG视频流
        ↓
[RobotIO线程] → 串口发送
```

#### 关键文件结构
```
script/
├── app.py              # Flask Web服务器
├── bridge.py           # C++/Python桥接函数
├── sensors_io.py       # 传感器启动脚本
├── autoaim.py          # 自瞄启动脚本
├── robot_io.py         # 通信启动脚本
└── templates/
    ├── index.html      # 参数调试界面
    └── video.html      # 视频显示页面
```

### Web UI改进方向

| 功能 | CVRM2021实现 | 可改进方向 |
|------|-------------|-----------|
| 视频流 | MJPEG (HTTP) | WebSocket + H.264 更低延迟 |
| 参数修改 | HTTP GET | WebSocket 双向实时 |
| 前端框架 | Bootstrap + 原生JS | Vue/React 更好交互 |
| 数据展示 | 静态列表 | ECharts 实时图表 |
| 配置持久化 | 无 | 保存到TOML文件 |

## Git Commit 规范

你是一个专业的 Git 提交信息生成助手。请严格按照以下规范生成 commit 信息。

### 基本格式
```
<emoji> <type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

### Emoji + Type 对照表

- ✨ `feat`: 新增功能
- 🐛 `fix`: 修复 bug
- 📝 `docs`: 文档更新
- 💄 `style`: 代码格式调整(不影响功能)
- ♻️ `refactor`: 代码重构(不增加功能,不修复bug)
- ⚡️ `perf`: 性能优化
- ✅ `test`: 测试相关
- 🔧 `chore`: 构建/工具/依赖更新
- 🔨 `build`: 构建系统修改
- 👷 `ci`: CI/CD 配置修改
- 💥 `BREAKING CHANGE`: 破坏性变更(使用感叹号!)

### 规则说明

1. **类型(必填)**: 使用上述 type 之一
2. **范围(可选)**: 用圆括号标注影响范围,如 `(api)` `(user)`
3. **描述(必填)**: 简短说明变更内容,建议不超过50字
4. **破坏性变更**:
   - 在类型后加 `!` 或在 footer 中使用 `BREAKING CHANGE:`
   - 必须说明影响和迁移方法
5. **正文(可选)**: 详细说明变更原因、内容
6. **页脚(可选)**: 关联 issue 或说明破坏性变更

### 示例

#### 示例1: 基础功能
```
✨ feat: 增加用户搜索功能
```

#### 示例2: 带范围和正文
```
✨ feat(notice): 增加消息搜索功能

1. 支持按关键词搜索
2. 搜索范围限制在近一个月
3. 支持模糊匹配
```

#### 示例3: 破坏性变更
```
🔨 build!: 升级依赖库版本

BREAKING CHANGE: 需要重新执行 npm install,Node 版本需 >=16
```

#### 示例4: 关联 issue
```
🐛 fix(auth): 修复登录超时问题

Closes: #123
```

#### 示例5: 完整格式
```
✨ feat(payment): 新增支付宝支付方式

功能详情:
1. 集成支付宝 SDK
2. 实现扫码支付流程
3. 添加支付状态回调

注意事项: 需要配置支付宝商户信息

BREAKING CHANGE: 支付接口参数结构调整,需更新调用方代码

Reviewed-by: 张三
Closes: #234, #235
```

### 生成要求

- 所有描述使用中文
- emoji 必须放在最前面
- 描述要简洁明确,一句话说清楚做了什么
- 如有破坏性变更,必须明确标注并说明影响
- 优先使用常用类型: feat, fix, docs, refactor, perf
- **提交信息保持简洁，严禁添加任何自动生成标记**：
  - 禁止添加 "🤖 Generated with [Claude Code](https://claude.com/claude-code)"
  - 禁止添加 "Co-Authored-By: Claude <noreply@anthropic.com>"
  - 禁止添加任何其他AI工具生成的标记
  - 只包含人为编写的提交内容

## 设计规划: Predictor 与火控数据结构

### 为什么要分线程？

与 rm.cv.fans（Predictor和火控在同一线程）不同，本项目分线程的原因：

1. **火控需要更高频率**: Predictor 约 30Hz（受相机帧率限制），火控想 100Hz 插值
2. **火控计算很重**: 弹道解算/MPC 耗时长，不想阻塞 Predictor

### 核心设计：时间戳 + 速度 → 插值

火控在 Predictor 两次更新之间（~33ms）需要自己做短期预测：

```cpp
// 火控插值逻辑
double dt = current_time - snapshot.timestamp;  // 关键：需要时间戳

// 装甲板位置插值
Eigen::Vector3d predicted_pos = armor.position + armor.velocity * dt;

// 陀螺相位插值
double predicted_phase = spin.phase + spin.omega * dt;
```

### 命名空间结构
```
namespace aimer::predictor {
    struct ArmorState;        // 单个装甲板状态
    struct SpinState;         // 陀螺运动状态
    struct VehicleState;      // 单车整体状态
    struct BattlefieldSnapshot; // 战场快照 (所有车)
}
```

### 核心数据结构

#### 1. ArmorState - 单个装甲板滤波状态
```cpp
struct ArmorState {
    int armor_id = -1;

    // 3D位置/速度 (世界坐标系) - 速度用于插值！
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;

    // 状态
    double last_seen_time = 0;
    bool visible = false;
};
```

#### 2. SpinState - 陀螺运动状态
```cpp
struct SpinState {
    bool active = false;
    double omega = 0;       // 角速度 (rad/s) - 用于相位插值
    double phase = 0;       // 当前相位 (rad)
    double radius = 0;      // 陀螺半径 (m)
};
```

#### 3. VehicleState - 单车整体状态
```cpp
struct VehicleState {
    int target_id = -1;
    EnemyType enemy_type;
    bool alive = false;

    // 整车中心 + 速度 (用于插值)
    Eigen::Vector3d center_pos;
    Eigen::Vector3d center_vel;

    // 陀螺状态 (omega用于相位插值)
    SpinState spin;

    // 装甲板状态 (位置+速度，用于插值)
    std::array<ArmorState, 4> armors;
    int armor_count = 0;
    int best_armor_idx = -1;

    // 时间戳 (关键！火控用于计算dt)
    double timestamp = 0;
};
```

#### 4. BattlefieldSnapshot - 战场快照
```cpp
constexpr int MAX_ENEMY_NUMBER = 8;

struct BattlefieldSnapshot {
    // 所有车辆状态 (索引 = 目标编号)
    std::array<VehicleState, MAX_ENEMY_NUMBER + 1> vehicles;

    // 位掩码
    uint16_t detection_mask = 0;  // 当前帧检测到的车
    uint16_t alive_mask = 0;      // 模型存活的车

    // 自身状态
    RobotState self_state;

    // 时间戳 (关键！)
    double timestamp = 0;
    int frame_id = 0;

    // 辅助方法
    bool is_alive(int id) const;
    const VehicleState& get(int id) const;

    template<typename Func>
    void for_each_alive(Func&& func) const;
};
```

### 数据流
```
Predictor (30Hz)                         火控 (100Hz)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
检测 + EKF更新
    ↓
导出 Snapshot (带时间戳+速度)
    ↓
UMT::push ─────────────────→ UMT::pop (非阻塞)
                                    ↓
                             dt = now - snapshot.timestamp
                                    ↓
                             pos' = pos + vel * dt (插值)
                             phase' = phase + omega * dt
                                    ↓
                             弹道解算 / MPC
                                    ↓
                             RobotCmd → 串口
```

### 使用示例
```cpp
// ========== Predictor (30Hz) ==========
umt::Publisher<BattlefieldSnapshot> pub("battlefield");

void predictor_loop() {
    // ... EKF更新 ...

    BattlefieldSnapshot snapshot;
    snapshot.timestamp = get_current_time();  // 关键！

    for (int i = 1; i <= 8; ++i) {
        auto& v = snapshot.vehicles[i];
        v.timestamp = snapshot.timestamp;

        // 从滤波器导出位置和速度
        v.center_pos = filter.predict_pos(0);
        v.center_vel = filter.predict_vel(0);  // 必须有速度！

        // 陀螺状态
        v.spin.omega = top_model.get_omega();
        v.spin.phase = top_model.get_phase();
    }

    pub.push(snapshot);
}

// ========== 火控 (100Hz) ==========
umt::Subscriber<BattlefieldSnapshot> sub("battlefield");

void fire_control_loop() {
    // 非阻塞获取最新快照
    auto snapshot = sub.pop_for(5);  // 5ms超时

    double now = get_current_time();
    double dt = now - snapshot.timestamp;  // 计算时间差

    // 选择目标
    int target_id = select_target(snapshot);
    const auto& target = snapshot.get(target_id);

    // 插值预测当前位置
    Eigen::Vector3d current_pos =
        target.center_pos + target.center_vel * dt;

    double current_phase =
        target.spin.phase + target.spin.omega * dt;

    // 弹道解算 / MPC
    auto cmd = solve_trajectory(current_pos, current_phase, ...);

    serial_send(cmd);
}
```

### 与 rm.cv.fans 的区别

| 方面 | rm.cv.fans | 本项目 |
|-----|-----------|--------|
| 架构 | 单线程，直接调用 | 分线程，消息传递 |
| 快照结构 | 无 | BattlefieldSnapshot |
| 火控频率 | 与Predictor相同 | 可独立更高 |
| 插值 | 不需要 | 需要（时间戳+速度） |

### 参考
- rm.cv.fans: `EnemyState[]` + `EnemyModelInterface*[]` 数组管理多车
- 关键区别: rm.cv.fans 不需要快照结构，因为同一线程直接调用