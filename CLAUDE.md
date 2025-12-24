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
// hardware/serial/serial_thread.hpp

// 接收数据结构
struct SerialReceiveData {
    // IMU 姿态数据
    float yaw;            // 偏航角 (°)
    float pitch;          // 俯仰角 (°)
    float roll;           // 横滚角 (°)

    // 机器人状态
    uint8_t robot_id;     // 机器人ID (1-7红方, 101-107蓝方)
    uint8_t enemy_color;  // 敌方颜色 (0=未知, 1=红, 2=蓝)

    // 射击参数
    float bullet_speed;   // 弹速 (m/s)

    // 模式控制
    uint8_t aim_mode;     // 自瞄模式 (0=关闭, 1=自瞄, 2=小符, 3=大符)
    bool allow_fire;      // 是否允许射击

    // 时间戳 (上位机接收时刻，微秒)
    int64_t recv_time_us = 0;
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

## Predictor Motion 模型架构

### 模块层次

```
aimer/auto_aim/predictor/
├── enemy_model/           # EnemyModelInterface 实现
│   ├── vehicle_model.*    # 车辆模型 (英雄/步兵/哨兵)
│   └── outpost_model.*    # 前哨站模型包装器
│
├── enemy_model/motion/    # 底层 EKF 运动模型
│   ├── armor_motion.*     # 单装甲板滤波 (多 EKF)
│   ├── spin_motion.*      # 整车旋转模型 (9维 EKF)
│   └── outpost_motion.*   # 前哨站模型 (7维 EKF)
│
└── enemy_state/
    └── armor_identifier.* # 跨帧装甲板 ID 分配
```

### Motion 模型对比

| 模型 | EKF 数量 | 状态维度 | 观测类型 | 适用目标 |
|------|---------|---------|---------|---------|
| `ArmorMotion` | 多个 (per ID) | 6维 YPD | YPD 直接观测 | 非陀螺目标 |
| `SpinMotion` | 1个 | 9维 XYZ | YPD 观测 | 陀螺车辆 (4装甲板) |
| `OutpostMotion` | 1个 | 7维 XYZ | YPD 观测 | 前哨站 (3装甲板) |

### EKF 状态与观测设计

**为什么 XYZ 状态 + YPD 观测？**

- **状态用 XYZ (笛卡尔)**：旋转中心预测是线性的 `x' = x + vx*dt`
- **观测用 YPD (球坐标)**：相机噪声在角度域更均匀，距离噪声与距离成正比

```cpp
// 状态向量 (SpinMotion 9维)
[xc, vx, yc, vy, zc, θ, ω, r, r2]
//  ↑ 旋转中心    ↑ 相位/角速度  ↑ 两个半径

// 观测向量 (4维)
[yaw, pitch, distance, armor_yaw]
```

### 前哨站模型 (OutpostMotion)

#### 规则参数 (2025)
```cpp
constexpr double OMEGA_ABS = 0.8 * M_PI;  // |ω| = 0.8π rad/s (固定)
constexpr double RADIUS = 0.553;           // 半径 0.553m (固定)
constexpr double DZ_STEP = 0.10;           // 高度差 10cm (固定)
```

#### 状态向量 (7维)
```cpp
[xc, vx, yc, vy, zc, θ, ω]
// xc,yc,zc: 旋转中心
// vx,vy: 中心速度 (前哨站可能在移动平台上)
// θ: 相位
// ω: 角速度 (约束 |ω| ≈ 0.8π，只需确定符号)
```

#### 快速收敛策略

**角速度方向快速判断**

|ω| = 0.8π 是已知的，只需判断符号（顺时针/逆时针）。

**策略：初始 ω=0，EKF 估计超过阈值后锁定**

```cpp
// init(): 初始为0
x0[outpost::OMEGA] = 0;

// constrain_omega(): 达到阈值后锁定方向
void OutpostMotion::constrain_omega() {
    double omega = x[outpost::OMEGA];

    if (!omega_sign_determined_) {
        constexpr double OMEGA_THRESHOLD = 0.4 * M_PI;
        if (omega > OMEGA_THRESHOLD) {
            x[outpost::OMEGA] = +0.8π;  // 逆时针
            omega_sign_determined_ = true;
        } else if (omega < -OMEGA_THRESHOLD) {
            x[outpost::OMEGA] = -0.8π;  // 顺时针
            omega_sign_determined_ = true;
        }
    } else {
        // 已确定，只约束绝对值
        x[outpost::OMEGA] = copysign(0.8π, omega);
    }
}
```

**优点：**
- 简单，利用 EKF 自身估计能力
- 不需要额外状态变量跟踪
- 装甲板切换也不影响

**2. 高度差学习**

三个槽位高度：{低, 中, 高} 间隔 10cm，但**排列顺序未知**。

```
可能排列 (6种):
槽位0=低, 槽位1=中, 槽位2=高  → [0, +10, +20] cm
槽位0=中, 槽位1=高, 槽位2=低  → [0, +10, -10] cm
...
```

**无法从2个观测推断第3个**，必须实际观测3个槽位才能完全学习。

```cpp
std::array<double, 3> slot_dz_;      // 各槽位高度差
std::array<bool, 3> slot_known_;     // 是否已观测到

// 只有 slot_known_[i] = true 的槽位才有准确高度
```

#### 盲区预测

前哨站可能短暂进入盲区（装甲板不可见），但 EKF 状态持续存在：

```cpp
// 即使没有新观测，也能预测位置
// 因为 θ, ω 状态持续更新 (predict without update)
Eigen::Vector3d predict_armor_pos(int slot, double dt) const {
    double theta = state.θ + state.ω * dt;  // 相位外推
    // ... 计算装甲板位置
}
```

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
namespace autoaim::predictor {
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
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // 用于火控插值

    double yaw = 0;            // 装甲板朝向 (rad)
    double z_to_v = 0;         // 相对相机的夹角，越小越正对

    int id = 0;                // 装甲板编号 0-3
    ArmorType type = ArmorType::SMALL;

    double score = 0;          // 打击评分 (0~1)
    bool visible = false;      // 当前帧是否可见
    double last_seen = 0;      // 上次看到的时间

    Eigen::Vector3d predict_position(double dt) const;
};
```

#### 2. SpinState - 陀螺运动状态
```cpp
struct SpinState {
    bool active = false;
    SpinLevel level = SpinLevel::NONE;  // NONE/LOW/HIGH

    double omega = 0;          // 角速度 (rad/s)，正值为逆时针
    double phase = 0;          // 当前相位 (rad)，即车体朝向角 θ
    double radius = 0;         // 陀螺半径 (m)
    double radius_2 = 0;       // 第二半径 (四装甲板时)

    double predict_phase(double dt) const;
    void update_level(double new_omega);  // 带迟滞消抖
    void reset();
};
```

#### 3. VehicleState - 单车整体状态
```cpp
struct VehicleState {
    int target_id = -1;
    EnemyType enemy_type = EnemyType::UNKNOWN;
    bool valid = false;

    // 旋转中心 + 速度 (用于插值)
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    // 陀螺状态
    SpinState spin;

    // 装甲板 (最多4块)
    std::array<ArmorState, MAX_ARMORS_PER_TARGET> armors;
    int armor_count = 4;

    // 置信度
    double confidence = 0;
    double position_std = 0;
    double velocity_std = 0;

    // 推荐目标
    int recommended_armor_idx = -1;

    // 时间戳
    double timestamp = 0;
    int frame_count = 0;

    // 辅助方法
    const ArmorState* get_recommended_armor() const;
    Eigen::Vector3d predict_center(double dt) const;
    Eigen::Vector3d predict_armor_position(int armor_idx, double dt) const;
};
```

#### 4. BattlefieldSnapshot - 战场快照
```cpp
// aimer/auto_aim/predictor/types.hpp
constexpr int MAX_TARGETS = 9;

struct BattlefieldSnapshot {
    std::array<VehicleState, MAX_TARGETS> vehicles;

    uint16_t valid_mask = 0;       // 哪些目标有效
    uint16_t detected_mask = 0;    // 当前帧检测到哪些

    int primary_target_id = -1;    // 主目标编号

    // 检测时刻的自身状态 (火控用)
    aimer::RobotState self_state;

    double timestamp = 0;
    int frame_id = 0;

    // 辅助方法
    bool is_valid(int id) const;
    bool is_detected(int id) const;
    const VehicleState& get(int id) const;
    const VehicleState* get_primary() const;
    void set_valid(int id, bool valid);
    void set_detected(int id, bool detected);
    void for_each_valid(Func&& func) const;
    void clear();
};
```

#### 5. 自身状态共享

`BattlefieldSnapshot.self_state` 包含检测时刻的自身状态，火控直接从 snapshot 读取：
```cpp
const auto& snapshot = battlefield->get();
const auto& self = snapshot.self_state;
// self.q_imu, self.bullet_speed, self.velocity ...
```

**注意**: `RobotState.velocity`（底盘速度）当前未赋值，需要：
1. 串口协议添加底盘速度 vx, vy
2. `RobotState::from_sync_frame()` 读取并赋值 velocity

### 数据流
```
Hardware (200Hz)
    ↓
SyncFrame (image + serial_data)
    ↓
Detector → DetectionResult (含 RobotState)
    ↓
Predictor (30Hz)
    ↓
┌───────────────────────────────────────────────────────────────┐
│ BasicObjManager<BattlefieldSnapshot>("battlefield")           │
│   - vehicles[]: 敌方状态                                       │
│   - self_state: 检测时刻的自身状态                              │
│                                                               │
│   火控 (100Hz) 多次读取同一帧数据做插值:                        │
│   dt = now - snapshot.timestamp                               │
│   pos' = pos + vel * dt                                       │
└───────────────────────────────────────────────────────────────┘
```

### 使用示例

> **注意**: 当前 predictor_node.cpp 仍使用 `Publisher`，待改为 `BasicObjManager`

```cpp
// ========== Predictor (30Hz) ==========
auto battlefield = umt::BasicObjManager<BattlefieldSnapshot>::find_or_create("battlefield");

void predictor_update(const DetectionResult& detection) {
    // ... EKF更新 ...
    // predict() 内部会设置 snapshot.self_state = detection.state
    auto snapshot = predictor.predict(detection, timestamp);
    battlefield->get() = snapshot;
}

// ========== 火控 (100Hz) ==========
auto battlefield = umt::BasicObjManager<BattlefieldSnapshot>::find("battlefield");

void fire_control_loop() {
    const auto& snapshot = battlefield->get();
    const auto& self = snapshot.self_state;  // 检测时刻的自身状态

    double now = get_current_time();
    double dt = now - snapshot.timestamp;

    // 选择目标并插值
    const auto* target = snapshot.get_primary();
    if (!target) return;

    Eigen::Vector3d predicted_pos = target->predict_center(dt);
    double predicted_phase = target->spin.predict_phase(dt);

    // 弹道解算
    auto cmd = solve_trajectory(
        predicted_pos,
        self.q_imu,          // 检测时刻的云台姿态
        self.bullet_speed,   // 弹速
        self.velocity        // 底盘速度 (动打动)
    );

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