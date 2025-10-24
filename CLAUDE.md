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
   - `Message.hpp` - Thread-safe message passing
   - `ObjManager.hpp` - Object lifecycle management
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
- **Serial Communication**: Supports both real hardware and simulated data for testing
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

### Debug Logging
```cpp
debug::init_md_file("log.log");  // Initialize markdown logger
debug::print("info", "module", "message: {}", data);
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