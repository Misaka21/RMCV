# Transformer 坐标变换模块

## 概述

无锁、纯函数设计的坐标变换模块。

## 架构

```
┌─────────────────────────────────────────────────────────────┐
│                    初始化 (main)                             │
│   BasicObjManager<TransformConfig> ← 相机内参、R_ic、T_cb    │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ 只读
              ┌───────────────┴───────────────┐
              ↓                               ↓
    ┌──────────────────┐            ┌──────────────────┐
    │  Predictor Thread │            │ Fire Control     │
    │                  │            │     Thread       │
    │  订阅: FrameData │            │  订阅: AimData   │
    │  (含配对四元数)   │            │  (含配对四元数)   │
    └──────────────────┘            └──────────────────┘
              │                               │
              │ tf::xxx(p, q, cfg)            │
              ↓                               ↓
         纯函数计算                       纯函数计算
```

## 坐标系定义

```
World (大地)  ←──[R_imu]──→  IMU (陀螺仪)  ←──[R_ic]──→  Camera (相机)  ←──[T_cb]──→  Barrel (枪管)
```

| 坐标系 | 后缀 | 说明 |
|--------|------|------|
| World | `_w` | 大地坐标系，Z轴朝上 |
| IMU | `_i` | 陀螺仪坐标系，随云台姿态变化 |
| Camera | `_c` | 相机坐标系，Z朝前，X朝右，Y朝下 |
| Barrel | `_b` | 枪管坐标系，以枪口为原点 |

## 快速开始

### 1. 初始化配置 (main.cpp)

```cpp
#include "aimer/common/transformer/transformer.hpp"
#include "umt/BasicObjManager.hpp"

void init_transformer() {
    auto cfg = umt::BasicObjManager<aimer::TransformConfig>::find_or_create(
        aimer::shared::TRANSFORM_CONFIG
    );

    // 从 YAML 加载相机内参和 R_ic
    cfg->get().load_from_yaml(CONFIG_DIR "/camera.yaml");
    cfg->get().T_cb = Eigen::Vector3d(0.0, 0.05, 0.1);  // 相机到枪管偏移
}
```

### 2. 在线程中使用

```cpp
void predictor_thread() {
    // 获取静态配置 (只读)
    auto cfg_obj = umt::BasicObjManager<aimer::TransformConfig>::find_or_create(
        aimer::shared::TRANSFORM_CONFIG
    );
    const auto& cfg = cfg_obj->get();

    // 订阅帧数据
    umt::Subscriber<FrameData> frame_sub("frame_data");

    while (running) {
        auto frame = frame_sub.pop();

        // 四元数从消息获取 (已与图像配对)
        Eigen::Quaterniond q_imu = frame.q_imu;

        // 检测目标
        Eigen::Vector3d target_camera = detect(frame.image);

        // 使用 tf:: 纯函数转换
        Eigen::Vector3d target_world = aimer::tf::camera_to_world(target_camera, q_imu, cfg);
    }
}
```

## API

### tf:: 纯函数

```cpp
// 基础变换
tf::imu_to_camera(p_i, R_ic)           // IMU -> Camera
tf::camera_to_imu(p_c, R_ic)           // Camera -> IMU
tf::camera_to_barrel(p_c, T_cb)        // Camera -> Barrel
tf::barrel_to_camera(p_b, T_cb)        // Barrel -> Camera
tf::world_to_imu(p_w, R_imu)           // World -> IMU
tf::imu_to_world(p_i, R_imu)           // IMU -> World

// 组合变换 (传入四元数)
tf::world_to_camera(p_w, q_imu, R_ic)
tf::camera_to_world(p_c, q_imu, R_ic)
tf::world_to_barrel(p_w, q_imu, R_ic, T_cb)
tf::barrel_to_world(p_b, q_imu, R_ic, T_cb)

// 便捷函数 (传入 TransformConfig)
tf::camera_to_world(p_c, q_imu, cfg)
tf::world_to_camera(p_w, q_imu, cfg)
tf::camera_to_barrel(p_c, cfg)
tf::world_to_barrel(p_w, q_imu, cfg)
tf::barrel_to_world(p_b, q_imu, cfg)

// XYZ <-> YPD
tf::xyz_to_ypd(xyz)   // -> YpdCoord
tf::ypd_to_xyz(ypd)   // -> Vec3
```

### YpdCoord

```cpp
// XYZ -> YPD
auto ypd = aimer::YpdCoord::from_xyz(p_barrel);

// YPD -> XYZ
Eigen::Vector3d xyz = ypd.to_xyz();

// 应用偏移
auto ypd_offset = ypd.with_offset_deg(1.5, -0.5);  // yaw偏1.5度, pitch偏-0.5度
```

## 完整示例

```cpp
#include "aimer/common/transformer/transformer.hpp"
#include "umt/BasicObjManager.hpp"
#include "umt/Message.hpp"

// 消息结构体
struct FrameData {
    cv::Mat image;
    Eigen::Quaterniond q_imu;  // 时间戳配对的四元数
};

// 预测器线程
void predictor_thread() {
    auto cfg_obj = umt::BasicObjManager<aimer::TransformConfig>::find_or_create(
        aimer::shared::TRANSFORM_CONFIG
    );
    const auto& cfg = cfg_obj->get();

    umt::Subscriber<FrameData> sub("frame_data");
    umt::Publisher<AimData> pub("aim_data");

    while (running) {
        auto frame = sub.pop();

        // 检测 -> 相机系坐标
        Eigen::Vector3d target_camera = detect(frame.image);

        // 相机系 -> 大地系
        Eigen::Vector3d target_world = aimer::tf::camera_to_world(
            target_camera, frame.q_imu, cfg
        );

        // 发布
        pub.push({target_world, frame.q_imu});
    }
}

// 火控线程
void fire_control_thread() {
    auto cfg_obj = umt::BasicObjManager<aimer::TransformConfig>::find_or_create(
        aimer::shared::TRANSFORM_CONFIG
    );
    const auto& cfg = cfg_obj->get();

    umt::Subscriber<AimData> sub("aim_data");

    while (running) {
        auto aim = sub.pop();

        // 大地系 -> 枪管系
        Eigen::Vector3d target_barrel = aimer::tf::world_to_barrel(
            aim.target_world, aim.q_imu, cfg
        );

        // 转 YPD
        auto ypd = aimer::tf::xyz_to_ypd(target_barrel);

        // 发送云台指令
        send_gimbal(ypd.yaw, ypd.pitch);
    }
}
```

## 设计要点

| 特性 | 说明 |
|------|------|
| **无锁** | 配置只读，四元数随消息传递 |
| **精准** | 四元数与图像时间戳配对 |
| **简单** | 纯函数，无全局状态 |
| **高效** | inline 函数，零开销 |
