# RMCV2026 AI 协作说明

本文件是仓库级入口说明。进入具体目录工作时，优先读取该目录下的
`CLAUDE.md`；同目录的 `AGENTS.md` 是给 Codex 使用的入口文件，会要求 Codex
读取同目录 `CLAUDE.md`。当前挂载点不允许创建软链接或硬链接，所以正文只维护在
`CLAUDE.md`。

## 项目定位

RMCV2026 是 RoboMaster 机器人视觉系统，使用 C++17 + CMake。主流程集成硬件
采集、串口通信、装甲板自瞄、能量机关、自身状态转换、预测、火控、日志、
录制和可视化。

运行主程序入口是 `main.cpp`。启动顺序大致为：

1. 初始化日志和运行时参数热重载。
2. 初始化坐标变换系统。
3. 启动硬件节点，发布 `hardware::SyncFrame` 到 `"sync_frame"`。
4. 启动自瞄 detector、predictor、fire_control。
5. 启动能量机关 detector、predictor、fire_control。
6. 启动 recorder、visualizer、watchdog。

## 目录说明

- `aimer/`: 业务算法层，包括自瞄、能量机关、通用坐标/弹道/状态类型。
- `hardware/`: 硬件层，只负责相机、串口、同步帧，不写业务策略。
- `plugin/`: 日志、参数、录制回放、可视化、统计、看门狗等基础设施。
- `umt/`: 线程间通信和共享对象管理。
- `config/`: 共享配置文件，运行时参数以 TOML 为主。
- `test/`: 可执行测试和离线验证工具。
- `scripts/`: systemd、watchdog、清理和模型预处理脚本。
- `simulator/`: 可选仿真节点。

## 构建与运行

常用构建命令：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j$(nproc)
```

Release 构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

运行：

```bash
./build/RMCV2026
./build/RMCV2026 --match
```

常用测试：

```bash
./build/test_param
./build/test_transformer
./build/test_serial
./build/test_camera
./build/test_fire_control
```

如果改动触及 CMake、公共头文件、线程通信、参数系统或主流程，至少执行一次完整
构建。

## 全局代码规范

- 使用 C++17，沿用现有 CMake 目标和目录结构。
- 缩进 4 空格，单行尽量不超过 100 字符。
- 类和结构体使用 `CamelCase`，函数、变量、命名空间使用 `lower_case`。
- 代码注释使用简洁中文；日志、调试 key、外部接口文本优先英文。
- 不为小改动引入新框架或大抽象，先复用现有 helper 和模块边界。
- 不要把硬件协议含义泄漏到业务层，也不要把业务枚举塞进硬件层。

## 运行时参数硬规则

运行时 TOML 参数必须在使用点读取：

```cpp
double q_pos = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.q_pos");
```

禁止在构造函数、`init()`、静态变量、helper 函数或配置结构体里缓存运行时参数。
这样会破坏热重载。

TOML 类型必须严格匹配：

- `2.8` 是 `double`
- `2` 是 `int64_t`
- `false` 是 `bool`

`get_param<double>()` 读取整数或布尔会失败并返回默认值。

## 线程通信约定

- 消息流使用 `umt::Publisher<T>` / `umt::Subscriber<T>`。
- 共享状态使用 `umt::BasicObjManager<T>`。
- 高频跨线程共享对象写入时优先使用已有 `store()` / `load()` 风格，避免读到半更新状态。
- 全局退出标志是 `BasicObjManager<bool>("app_running")`。
- 重要线程需要向 watchdog 打心跳。

关键通道：

- `"sync_frame"`: `hardware::SyncFrame`
- `"detections"`: 自瞄检测结果
- `"battlefield"`: 自瞄预测快照
- `"fire_command"`: 火控输出给串口的命令
- `"match_mode"`: 比赛模式标志

## 分层边界

硬件层只处理原始数据和同步：

```text
hardware/serial -> hardware::SyncFrame -> aimer/common::RobotState -> detector/predictor/fire_control
```

`hardware/serial` 中的 `aim_mode` 是 `uint8_t` 原始字节。
业务含义在 `aimer/common/robot_state.hpp` 中转换为 `aimer::AimMode`。

## 协作分支建议

- `master`: 稳定版本，只通过 PR/MR 合并。
- `dev`: 大改集成分支。
- `feat/<name>` / `fix/<name>` / `refactor/<name>`: 个人工作分支。

公共接口、`config/*.toml`、`CMakeLists.txt`、`main.cpp`、`plugin/param`、`umt`
改动前先同步改动范围。

## 提交信息

提交、生成 commit message 或整理提交说明时使用 `$rmcv-git-commit` skill。
不要添加 AI 工具自动生成标记或 `Co-Authored-By`。
