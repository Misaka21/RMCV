# aimer/common 说明

本目录保存业务层共享能力。这里的代码被自瞄和能量机关共同依赖，改动影响面较大。

## 主要内容

- `robot_state.hpp`: 从 `hardware::SyncFrame` 转换出的业务机器人状态。
- `types.hpp` / `fire_control_types.hpp`: 跨模块共享类型。
- `transformer/`: 坐标系转换系统。
- `trajectory/` 和 `ballistic/`: 弹道解算。
- `filter/`: EKF 等滤波工具。
- `latency/`: 延迟估计。
- `math/`: 通用数学函数。

## 分层规则

- `AimMode` 等业务枚举定义在这里，不放进 `hardware/serial`。
- `RobotState::from_sync_frame()` 是硬件层到业务层的转换边界。
- 通用工具不能依赖 auto_aim 或 auto_buff 的内部类型。
- 对外公共头文件要尽量稳定，修改后检查两个业务链路。

## 坐标约定

常用坐标系：

- `World`: 上电时云台位置，X 前、Y 左、Z 上。
- `Imu`: IMU 芯片坐标系，按硬件定义。
- `Gimbal`: 云台坐标系，X 前、Y 左、Z 上。
- `Camera`: 相机坐标系，X 右、Y 下、Z 前。
- `Barrel`: 枪口坐标系，X 前、Y 左、Z 上。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_transformer
./build/test_ballistic
./build/test_ground_plane
```
