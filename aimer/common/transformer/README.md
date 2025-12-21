# TF 坐标变换系统

编译期路径推导 + 运行期动态参数的坐标变换系统。

## 1. 坐标系定义

### 1.1 World (大地坐标系)

**上电时刻的云台坐标系**，作为固定参考系用于预测和跟踪。

```
          z (上)
          │
    y ←───┼───→ x (前，上电时云台朝向)
   (左)   │
```

- **原点**：上电时刻云台位置（后续通过里程计积分更新）
- **X轴**：上电时云台前方（枪管朝向）
- **Y轴**：上电时云台左侧
- **Z轴**：上电时云台上方
- **特点**：与 Gimbal 坐标系方向定义一致，上电时 `q_imu = Identity`

**为什么这样定义？**

1. 不需要绝对定位设备（GPS等）
2. 可以追踪机器人相对于起点的位移
3. 预测时可以考虑自身运动补偿

### 1.2 Imu (IMU芯片坐标系)

IMU 硬件定义的坐标系，可能因为安装方式而"歪着"。

- **原点**：IMU 芯片位置
- **方向**：由 IMU 硬件定义
- **动态**：通过四元数 `q_imu` 描述相对于 World 的姿态

### 1.3 Gimbal (云台坐标系)

修正 IMU 安装偏差后的云台坐标系，是我们真正想要的"云台姿态"。

```
          z (上)
          │
    y ←───┼───→ x (前，云台/枪管朝向)
   (左)   │
```

- **原点**：云台旋转中心
- **X轴**：云台前方（枪管朝向敌人）
- **Y轴**：云台左侧
- **Z轴**：云台上方

**为什么需要 Gimbal？**

IMU 可能"侧着装"在云台上：

```
正常安装:                    侧着装:
  IMU ──→ X                    IMU
   │                            │
   ↓ Y                          ↓ X
                               ──→ Y
```

`R_gimbal2imubody` 用于修正这个安装偏差。

### 1.4 Camera (相机坐标系)

相机光心为原点，遵循计算机视觉惯例。

```
              Z (前，光轴方向)
             ╱
            ╱
           ╱
          └──────→ X (右，图像u方向)
          │
          │
          ↓ Y (下，图像v方向)
```

- **原点**：相机光心
- **X轴**：图像右侧 (u 增大方向)
- **Y轴**：图像下侧 (v 增大方向)
- **Z轴**：光轴方向（前方）

### 1.5 Barrel (枪口坐标系)

弹道计算的起点。

- **原点**：枪口中心
- **方向**：与 Gimbal 相同

---

## 2. TF 树结构

```
World (大地坐标系)
  │
  │ ← q_imu (IMU四元数，动态)
  ▼
Imu (IMU芯片坐标系)
  │
  │ ← R_gimbal2imubody (静态，YAML)
  ▼
Gimbal (云台坐标系)
  │
  ├─────────────────────────────┐
  │                             │
  │ ← R_camera2gimbal (静态)    │ ← t_barrel2gimbal (动态)
  │ ← t_camera2gimbal (动态)    │
  ▼                             ▼
Camera                        Barrel
```

---

## 3. 变换参数含义

### 3.1 平移向量的含义

`t_A2B = [x, y, z]` 表示：

> **A 的原点** 在 **B 坐标系** 下的坐标

例如 `t_camera2gimbal = [0, 0.05, 0]`：
- Camera 原点在 Gimbal 坐标系下是 `(0, 0.05, 0)`
- 即：相机在云台**下方 5cm**（Y正=下）

```
Gimbal 原点 ●────────────────
            │
            │ 5cm (Y方向)
            ▼
Camera 原点 ●────────────────
```

### 3.2 旋转矩阵的含义

`R_A2B` 表示：

> 将 **A 坐标系的向量** 变换到 **B 坐标系**

### 3.3 齐次变换矩阵

```
T_A2B = [ R_A2B  t_A2B ]
        [  0      1    ]
```

将 A 坐标系下的点变换到 B 坐标系：

```
p_B = T_A2B * p_A
```

---

## 4. 配置文件

### 4.1 YAML (静态标定，启动时加载一次)

`config/camera.yaml`:

```yaml
# IMU 安装偏差修正 (Gimbal → Imu)
# 如果 IMU 装正了，就是单位阵
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]

# 相机安装角度 (Camera → Gimbal)
# 如果相机正对前方，就是单位阵
R_camera2gimbal: [1, 0, 0, 0, 1, 0, 0, 0, 1]

# 相机内参
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distort_coeffs: [k1, k2, p1, p2, k3]
```

### 4.2 TOML (动态调参，运行时可热更新)

`config/aimer.toml`:

```toml
[Transformer]
# Camera → Gimbal 平移 (米)
# 坐标系: x前, y左, z上 (ROS惯例)
camera_offset_x = 0.0    # 前为正
camera_offset_y = 0.0    # 左为正
camera_offset_z = 0.0    # 上为正

# Barrel → Gimbal 平移 (米)
barrel_offset_x = 0.0    # 前为正
barrel_offset_y = 0.0    # 左为正
barrel_offset_z = -0.055 # 上为正 (枪口在云台下方5.5cm)
```

---

## 5. 使用方法

### 5.1 初始化

```cpp
#include "aimer/common/transformer/transformer.hpp"

// 启动时调用一次
tf::init();  // 默认读取 CONFIG_DIR/camera.yaml
// 动态参数会在每次变换时自动从 TOML 读取
```

### 5.2 坐标变换

```cpp
// PnP 求解得到目标在相机坐标系下的位置
Eigen::Vector3d p_cam = pnp_solve(armor);

// 从串口获取 IMU 四元数
Eigen::Quaterniond q_imu = get_imu_quaternion();

// 变换到世界坐标系 (用于预测)
Eigen::Vector3d p_world = tf::cam_to_world(p_cam, q_imu);

// 变换到枪口坐标系 (用于弹道计算)
Eigen::Vector3d p_barrel = tf::world_to_barrel(p_world, q_imu);

// 或者直接 Camera → Barrel
Eigen::Vector3d p_barrel2 = tf::cam_to_barrel(p_cam, q_imu);
```

### 5.3 变换链

```cpp
// 自动推导路径: Camera → Gimbal → Imu → World
p_world = tf::point<tf::Frame::Camera, tf::Frame::World>(p_cam, q_imu);

// 变换向量 (速度/方向，只应用旋转，不应用平移)
v_world = tf::vector<tf::Frame::Camera, tf::Frame::World>(v_cam, q_imu);

// 获取变换矩阵
Eigen::Matrix4d T = tf::matrix<tf::Frame::Camera, tf::Frame::World>(q_imu);
```

---

## 6. 完整数据流

```
下位机数据 (Imu坐标系)          相机数据 (Camera坐标系)
├── vx, vy (速度)               ├── 装甲板像素位置
├── q_imu (姿态)                └── PnP → p_camera
│
└──────────────┬────────────────────────┘
               │
               ▼
         变换到 World 坐标系
         ├── v_world = tf::vector<Imu, World>(v_imu, q)
         └── p_world = tf::point<Camera, World>(p_cam, q)
               │
         大地坐标系下预测
         └── 考虑双方速度、重力、弹道
               │
               ▼
         变换回 Barrel 坐标系
         └── p_barrel = tf::point<World, Barrel>(p_predict, q)
               │
               ▼
         计算云台角度 (yaw, pitch)
```

---

## 6.1 里程计积分

里程计速度 `vx, vy` 在 **Gimbal 坐标系**下，需要转换到 World 后积分更新位置。

### 使用方法

```cpp
// 收到里程计数据时调用
tf::update_odometry(Eigen::Vector3d(vx, vy, 0), dt, q_imu);

// 获取机器人在World中的位置
Eigen::Vector3d robot_pos = tf::get_robot_position();

// 重置位置（如需要）
tf::reset_odometry();
```

### 原理

```
里程计数据 (Gimbal坐标系)
├── vx (前为正，枪管朝向)
├── vy (左为正)
└── vz (上为正，通常为0)
        │
        ▼
  转换到 World 坐标系
  v_world = tf::vector<Gimbal, World>(v_gimbal, q_imu)
        │
        ▼
  积分更新位置
  robot_position += v_world * dt
```

### 注意事项

1. **积分漂移**：里程计有累积误差，长时间运行会漂移
2. **时间同步**：`dt` 需要准确，建议使用时间戳计算
3. **初始位置**：上电时 `robot_position = (0, 0, 0)`
4. **用途**：用于预测时补偿自身运动

---

## 7. 常见问题

### Q1: 枪口在相机下方，barrel_offset_z 是正还是负？

**负**。因为 Z 轴向上，枪口在下方意味着 Z 坐标减小。

```
Gimbal 原点 ●
            │
            │ -Z (向下)
            ▼
Barrel 原点 ●

barrel_offset_z = -0.055  (下方5.5cm)
```

### Q2: 相机在云台前方，camera_offset_x 是正还是负？

**正**。X 轴向前，相机在前方意味着 X 坐标增大。

### Q3: R_gimbal2imubody 是什么？

修正 IMU 安装角度偏差的旋转矩阵。

如果 IMU 侧着装（比如 Z 轴朝右而不是朝前），需要用这个矩阵修正，使得 Gimbal 坐标系的 Z 轴正确指向前方。

### Q4: 变换方向搞反了怎么办？

使用模板参数指定方向：

```cpp
// Camera → World
p_world = tf::point<tf::Frame::Camera, tf::Frame::World>(p_cam, q);

// World → Camera (反向)
p_cam = tf::point<tf::Frame::World, tf::Frame::Camera>(p_world, q);
```

系统会自动处理逆变换。

### Q5: 多线程安全吗？

**安全**。
- 静态参数启动时设置一次，之后只读
- 动态参数通过 `runtime_param::get_param()` 读取
- `get()` 每次调用独立计算，不共享状态

### Q6: 上电时 World 和 Gimbal 是什么关系？

**完全重合**。

上电时刻：
- `q_imu = Identity`（四元数为单位四元数）
- World 坐标系 = Gimbal 坐标系
- `robot_position_in_world = (0, 0, 0)`

随着云台转动：
- `q_imu` 更新，描述 Gimbal 相对于 World 的姿态变化
- 通过里程计积分更新 `robot_position_in_world`

### Q7: 里程计的 vx, vy 分别是什么方向？

在 **Gimbal 坐标系**下：
- `vx`：前为正（X轴方向，枪管朝向）
- `vy`：左为正（Y轴方向）
- `vz`：上为正（Z轴方向，通常为0）

注意：这是云台坐标系下的速度，不是底盘坐标系。

---

## 8. 坐标系速查表

| 坐标系 | 原点 | X轴 | Y轴 | Z轴 |
|--------|------|-----|-----|-----|
| World | 上电时云台位置 | 前 | 左 | 上 |
| Imu | IMU芯片 | (由硬件定义) | | |
| Gimbal | 云台中心 | 前 | 左 | 上 |
| Camera | 相机光心 | 右(u) | 下(v) | 前(光轴) |
| Barrel | 枪口中心 | 前 | 左 | 上 |

**注意**：Camera 坐标系保持 OpenCV 惯例 (x右 y下 z前)，其他均为 ROS 惯例 (x前 y左 z上)。

---

## 9. 参数配置速查

| 参数 | 来源 | 含义 | 例子 |
|------|------|------|------|
| `R_gimbal2imubody` | YAML | IMU安装偏差 | 单位阵=装正 |
| `R_camera2gimbal` | YAML | 相机坐标系转换 | OpenCV→ROS |
| `camera_offset_x` | TOML | 相机在云台前方距离 | 0.02 = 前2cm |
| `camera_offset_y` | TOML | 相机在云台左侧距离 | 0.01 = 左1cm |
| `camera_offset_z` | TOML | 相机在云台上方距离 | 0.05 = 上5cm |
| `barrel_offset_x` | TOML | 枪口在云台前方距离 | |
| `barrel_offset_y` | TOML | 枪口在云台左侧距离 | |
| `barrel_offset_z` | TOML | 枪口在云台上方距离 | -0.055 = 下5.5cm |
