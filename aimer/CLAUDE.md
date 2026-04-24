# aimer 模块说明

`aimer/` 是业务算法层。这里可以理解串口数据的业务含义，可以定义自瞄模式、
目标类型、坐标系、弹道和火控策略，但不要直接操作底层相机或串口协议细节。

## 子模块

- `common/`: 通用类型、机器人状态、坐标变换、弹道、滤波、延迟估计。
- `auto_aim/`: 装甲板自瞄，包含 detector、predictor、fire_control。
- `auto_buff/`: 能量机关，包含 detector、predictor、fire_control。

## 数据流

```text
hardware::SyncFrame
    -> auto_aim/detector 或 auto_buff/detector
    -> predictor
    -> fire_control
    -> hardware/serial::VisionData_t
```

`hardware::SyncFrame` 中的串口字段是底层输入。业务层需要先通过
`aimer::RobotState::from_sync_frame()` 或等价 helper 转成业务状态。

## 边界规则

- 业务层可以依赖 `hardware::SyncFrame` 作为输入边界，但业务类型应定义在
  `aimer/common` 或具体业务模块中。
- 自瞄和能量机关可以共享 `aimer/common`，不要互相依赖对方内部实现。
- 坐标系、弹道、延迟估计等跨业务能力放在 `aimer/common`。
- 修改 `AimMode`、`RobotState`、火控公共类型时，检查自瞄、能量机关、串口和测试。

## 验证

常用命令：

```bash
cmake --build build -j$(nproc)
./build/test_transformer
./build/test_ballistic
./build/test_fire_control
```
