# 数据流与结构体说明

## 数据流架构

```
┌─────────────────┐     ┌─────────────────┐
│   相机采集线程    │     │   串口接收线程    │
│   (HikCam)      │     │  (serial_recv)  │
└────────┬────────┘     └────────┬────────┘
         │ cv::Mat                │ SerialReceiveData
         │ timestamp              │ recv_time
         │                        │
         └──────────┬─────────────┘
                    │ 时间同步匹配
                    ▼
         ┌─────────────────────┐
         │    SyncFrame 打包    │
         │  image + serial_data│
         └──────────┬──────────┘
                    │ UMT Publisher
                    │ "sync_frame"
                    ▼
         ┌─────────────────────┐
         │   DetectorNode      │
         │  订阅 SyncFrame      │
         │  获取图像和颜色等      │
         └──────────┬──────────┘
                    │ 检测
                    ▼
         ┌─────────────────────┐
         │  DetectionResult    │
         │  armors + 时间戳     │
         └──────────┬──────────┘
                    │ UMT Publisher
                    │ "detections"
                    ▼
                  后续处理...
```

## 核心结构体

### 1. SerialReceiveData - 串口接收数据

从电控接收的完整数据包。

```cpp
// hardware/serial/serial_thread.hpp
struct SerialReceiveData {
    // IMU 姿态数据
    float yaw;            // 偏航角 (rad)
    float pitch;          // 俯仰角 (rad)
    float roll;           // 横滚角 (rad)

    // 射击参数
    float bullet_speed;   // 弹速 (m/s)

    // 模式控制 (原始字节，业务含义见 aimer::AimMode)
    uint8_t aim_mode;     // 原始值 (0/1/2/3)，转换见 aimer::to_aim_mode()
    bool aiming_lock;     // 预瞄锁定 (右键按下=true, 释放=false)

    // 协议字段
    uint8_t enemy_color;  // 敌方颜色 (0=未知, 1=红, 2=蓝)

    // 本地软门控字段，不走当前串口协议
    bool allow_fire;      // 是否允许射击

    // 上位机接收时间戳
    int64_t recv_time_us;
};
```

### 2. ImuData - IMU数据（支持四元数）

从 SerialReceiveData 提取的 IMU 数据，支持四元数和旋转矩阵转换。

```cpp
// hardware/hardware_node.hpp
struct ImuData {
    float yaw   = 0;  // 偏航角 (°)
    float pitch = 0;  // 俯仰角 (°)
    float roll  = 0;  // 横滚角 (°)

    // 欧拉角转四元数 (ZYX顺序)
    Eigen::Quaterniond to_quaternion() const;

    // 获取旋转矩阵
    Eigen::Matrix3d to_rotation_matrix() const;
};
```

### 3. SyncFrame - 同步帧

相机图像与串口数据的同步打包。

```cpp
// hardware/hardware_node.hpp
struct SyncFrame {
    // 图像数据
    cv::Mat image;
    int frame_id = 0;
    TimePoint timestamp;  // 图像采集时间

    // 串口数据 (与图像时间同步)
    serial::SerialReceiveData serial_data;
    bool serial_valid = false;  // 串口数据是否有效

    // 便捷获取IMU数据
    ImuData imu() const;
};
```

### 4. DetectionResult - 检测结果

检测器输出的结果。

```cpp
// aimer/auto_aim/detector/detector_node.hpp
struct DetectionResult {
    int frame_id = 0;
    TimePoint timestamp;
    std::vector<detector::Armor> armors;
    float detect_latency_ms = 0;
};
```

## 使用示例

### 订阅 SyncFrame

```cpp
#include "hardware/hardware_node.hpp"
#include "umt/umt.hpp"

umt::Subscriber<hardware::SyncFrame> sub("sync_frame");

while (true) {
    auto frame = sub.pop_for(1000);  // 1秒超时

    // 获取图像
    cv::Mat img = frame.image;

    // 获取IMU数据
    if (frame.serial_valid) {
        auto imu = frame.imu();
        float yaw = imu.yaw;
        float pitch = imu.pitch;

        // 获取四元数
        Eigen::Quaterniond q = imu.to_quaternion();

        // 获取旋转矩阵
        Eigen::Matrix3d R = imu.to_rotation_matrix();
    }

    // 获取敌方颜色
    uint8_t color = frame.serial_data.enemy_color;  // 0=未知, 1=红, 2=蓝

    // 获取弹速
    float bullet_speed = frame.serial_data.bullet_speed;
}
```

### 颜色获取逻辑

DetectorNode 中的颜色获取优先级：

```cpp
// 默认使用串口 byte[19]；若 hardware.toml 中调试覆盖开启，则使用覆盖值。
if (frame.serial_data.enemy_color == 1) {
    detector->set_enemy_color(detector::EnemyColor::RED);
} else if (frame.serial_data.enemy_color == 2) {
    detector->set_enemy_color(detector::EnemyColor::BLUE);
}
```

## UMT 消息通道

| 通道名 | 数据类型 | 发布者 | 订阅者 |
|--------|----------|--------|--------|
| `sync_frame` | `SyncFrame` | HardwareNode | DetectorNode |
| `detections` | `DetectionResult` | DetectorNode | Predictor/Aimer |
| `serial_receive` | `SerialReceiveData` | 串口接收线程 | HardwareNode |

## 时间同步机制

HardwareNode 使用缓冲区进行时间同步匹配：

1. 串口接收线程将数据发布到 `serial_receive`
2. HardwareNode 主循环将串口数据转移到时间同步缓冲区
3. 相机采集图像后，根据 `delta_t_us` 配置找到最接近的串口数据
4. 打包成 SyncFrame 发布

### delta_t_us 参数说明

```toml
# hardware.toml
[TimeSync]
    # 相机-IMU时间偏移 (微秒)
    # 正值: 图像比IMU数据晚到达，匹配过去的IMU
    # 负值: 图像比IMU数据早到达，匹配未来的IMU
    # 例: delta_t_us = 5000 表示图像实际对应5ms前的IMU姿态
    delta_t_us = 5000
```

**时间线示意图（delta_t_us = 5000）：**

```
时间线:  ─────────────────────────────────────────>
              │                           │
         IMU数据接收                   图像接收
         (T-5ms时刻的姿态)             (T时刻)
              │                           │
              └───── delta_t_us = 5ms ────┘
                     (正值，往过去找)
```

**代码逻辑：**
```cpp
// target = 图像时间 - delta_t_us
auto target = cam_time - std::chrono::microseconds(delta_t_us);
// 在缓冲区中找最接近 target 的串口数据
auto data = find_closest_serial_data(serial_buffer, target);
```

**常见情况：**
- 相机有曝光延迟 + USB传输延迟，图像到达时实际场景已经是"过去"
- IMU通过串口几乎实时到达
- 因此 `delta_t_us` 通常为**正值**，表示图像对应过去某时刻的IMU姿态

## Fake Serial 配置

不接串口时使用虚拟数据进行测试。

```toml
# hardware.toml
[Serial]
    use_fake_serial_data = true  # 启用虚拟数据

[Serial.fake_data]
    # IMU姿态
    yaw_rad = 0.0
    pitch_rad = 0.0
    roll_rad = 0.0
    # 敌方颜色
    enemy_color = 1        # 0=未知, 1=红, 2=蓝
    # 射击参数
    bullet_speed = 15.0    # m/s
    # 模式控制 (原始值，对应 aimer::AimMode 枚举)
    aim_mode = 1           # 0=DISABLED, 1=AUTOAIM, 2=ENERGY_SMALL, 3=ENERGY_LARGE
    allow_fire = true
    aiming_lock = false

[Serial.enemy_color_override]
    enable = false         # 调试时才设为 true
    enemy_color = 1        # 0=未知, 1=红, 2=蓝
```

启用后，HardwareNode 会跳过串口初始化，直接使用配置的虚拟数据填充 `SyncFrame.serial_data`。
