# RMCV

RoboMaster Computer Vision - 机器人视觉系统

## 项目简介

RMCV 是一个用于 RoboMaster 机器人的视觉识别与自动瞄准系统，采用 C++17 开发。

## 主要模块

| 模块 | 说明 |
|-----|------|
| `hardware/` | 硬件抽象层 (相机、串口) |
| `aimer/` | 自动瞄准系统 |
| `plugin/` | 插件系统 (日志、参数管理) |
| `umt/` | 线程间通信框架 |

## 编译

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

## 依赖

- CMake 3.16+
- OpenCV
- fmt
- Eigen3
- tomlplusplus
- pybind11
- HIK MVS SDK

## 文档

详细开发文档请参考各模块的头文件注释。

## 许可证

MIT License
