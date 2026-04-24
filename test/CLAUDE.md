# test 目录说明

本目录是模块验证工具集合，不是单一测试框架。新增测试时优先保持一个清晰的可执行目标。

## 常用测试

- `test_param`: 参数系统。
- `test_transformer`: 坐标变换。
- `test_camera`: 相机。
- `test_serial`: 串口。
- `test_hardware`: 硬件节点。
- `test_fire_control`: 自瞄火控。
- `test_ballistic`: 弹道。
- `test_playback`: 回放。
- `time_sync/`: 相机和 IMU 时间戳验证。
- `extrinsic_calib/`: 外参标定。

## 改动规则

- 修 bug 时优先补能复现问题的最小测试或离线验证工具。
- 依赖 Ceres、TensorRT、ROS2 的测试要保持可选，不应破坏普通机器构建。
- 测试目标命名要能看出用途，加入根 `CMakeLists.txt`。

## 运行

```bash
cmake --build build -j$(nproc)
./build/test_param
./build/test_transformer
```
