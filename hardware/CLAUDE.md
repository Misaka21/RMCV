# hardware 模块说明

`hardware/` 是硬件抽象层，负责相机、串口、同步帧和硬件节点启动。这里不要写自瞄、
能量机关、目标选择或火控策略。

## 主要内容

- `hardware_node.*`: 启动相机和串口，发布同步帧。
- `hik_cam/`: 海康相机封装。
- `serial/`: 串口/USB Bulk 协议、收发线程和固定包。

## 输出边界

硬件层向业务层输出：

```cpp
hardware::SyncFrame
```

消息通道：

```text
Message<hardware::SyncFrame>("sync_frame")
```

`SyncFrame` 包含图像、帧号、时间戳、串口原始数据和串口有效标志。

## 分层规则

- 硬件层可以知道协议字段，但不解释业务策略。
- 串口层的 `aim_mode` 是 `uint8_t` 原始值，不是 `aimer::AimMode`。
- 敌方颜色、弹速、模式、预瞄锁定等字段可以透传给业务层。
- 硬件线程需要响应 `app_running` 退出。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_hardware
./build/test_camera
./build/test_serial
```
