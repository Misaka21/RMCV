# hik_cam 模块说明

本目录封装海康相机。它负责设备枚举、打开相机、取流、时间戳和异常降级，不负责业务
检测和目标选择。

## 主要文件

- `hik_camera.*`: 相机主实现。
- `hik_camera_fallback.cpp`: 无 SDK 或不可用时的 fallback。
- `hik_log.hpp`: 海康 SDK 日志辅助。

## 改动规则

- 保持没有海康 SDK 的机器仍能通过 fallback 构建。
- 相机输出时间戳必须能和串口数据对齐。
- 不要在相机层读取自瞄或能量机关参数。
- 相机配置和标定文件路径应通过配置或编译期目录宏处理。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_camera
./build/test_timestamp
./build/check_timestamp_order
```
