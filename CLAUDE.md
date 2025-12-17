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