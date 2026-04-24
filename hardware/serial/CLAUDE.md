# serial 模块说明

本目录只负责字节协议、收发线程、断线重连和数据包转换。业务含义在 `aimer/` 层解释。

## 核心文件

- `serial_thread.*`: 收发线程和协议结构体。
- `transceiver_manager.hpp`: 协议收发管理。
- `fixed_packet.hpp`: 固定长度包、帧头帧尾和 CRC。
- `protocol/uart_protocol.*`: UART 实现。
- `protocol/usb_bulk_protocol.*`: USB Bulk 实现。

## 协议结构

当前使用 32 字节固定包。

视觉到电控：

```cpp
serial::VisionData_t
```

电控到视觉：

```cpp
serial::SerialReceiveData
```

`SerialReceiveData::aim_mode` 是原始 `uint8_t`。转换为业务枚举的位置是
`aimer/common/robot_state.hpp` 中的 `to_aim_mode()`。

## 分层硬规则

```text
hardware/serial: 只管字节、校验、收发、原始字段
aimer/common:   定义 AimMode 等业务枚举
auto_aim/buff:  根据 AimMode 决定业务行为
```

不要在 `hardware/serial` 中包含 `aimer/common/robot_state.hpp`，也不要让串口层依赖
自瞄或能量机关模块。

## 修改协议时

必须同步检查：

- `VisionData_t`
- `SerialReceiveData`
- `SerialUtils::vision_data_to_packet()`
- `SerialUtils::packet_to_receive_data()`
- 电控端协议文档或代码
- `RobotState::from_sync_frame()`
- `test_serial`

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_serial
```
