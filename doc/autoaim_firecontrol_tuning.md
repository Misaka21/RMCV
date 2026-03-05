# AutoAim 火控调参手册（RMCV 当前实现）

> 目标：第一次接手代码的人，按本文可以完成符号校准、落点校准、延迟校准（1/2/3 测试）。
>
> 适用版本：`master`（含 `AimOffset` 角度制、枪口原点弹道输入修复）。

---

## 0. 先记住这 3 条

1. 所有串口角度都是 **弧度**（`f32`）。
2. 火控约定：**yaw 左正，pitch 上正**。
3. `AutoAim.FireControl.AimOffset.yaw/pitch` 在 `aimer.toml` 里是 **角度制 deg**。

---

## 1. 参数与单位总表（必须先统一）

### 1.1 串口协议（电控 <-> 视觉）

见 [serial_protocol_ec_32B.md](/Users/david/Documents/GitHub/RMCV/docs/serial_protocol_ec_32B.md)：

- 电控 -> 视觉（状态）：
  - `yaw/pitch/roll`：弧度，`f32`
- 视觉 -> 电控（指令）：
  - `yaw/pitch`：弧度，`f32`

### 1.2 关键配置文件

- 火控/预测参数：`/Users/david/Documents/GitHub/RMCV/config/aimer.toml`
- 串口与 IMU 符号修正：`/Users/david/Documents/GitHub/RMCV/config/hardware.toml`

### 1.3 当前实现的符号定义

- 世界/云台坐标：`x前 y左 z上`
- `yaw = atan2(y, x)`：左转增大
- `pitch = atan2(z, sqrt(x^2+y^2))`：抬头增大

### 1.4 枪口/相机外参的正负方向（Transformer）

`aimer.toml -> [Transformer]`：

- `camera_offset_x/y/z`：相机在云台坐标系下的位置（前/左/上为正）
- `barrel_offset_x/y/z`：枪口在云台坐标系下的位置（前/左/上为正）

常见例子：

- 枪口在云台下方 5.5cm：`barrel_offset_z = -0.055`
- 相机在云台右侧 6.8mm：`camera_offset_y = -0.0068`

---

## 2. Test 1：符号/坐标联调（不开火）

目的：确保“电控上报角度”和“视觉下发角度”与代码同号，避免后续所有调参反着来。

### 2.1 要检查的参数

`hardware.toml`：

```toml
[Serial]
imu_yaw_negate = false
imu_pitch_negate = true
imu_roll_negate = false
```

这三个开关是“最后一道符号修正”，用于把电控上报 IMU 角度修正到本项目标准（左正、上正）。

### 2.2 操作步骤

1. 启动系统，打开 `Visualizer.view = "firecontrol"`。
2. 先不打弹，手动控制云台：
   - 云台向左转一点，再回中；
   - 云台向上抬一点，再回中。
3. 观察左上角面板 `Gimbal: yaw/pitch`：
   - 左转时 `yaw` 应增大；
   - 抬头时 `pitch` 应增大。

### 2.3 判定与处理

- 若左转时 `yaw` 变小：把 `imu_yaw_negate` 翻转。
- 若抬头时 `pitch` 变小：把 `imu_pitch_negate` 翻转。
- 若右倾时 `roll` 变小：把 `imu_roll_negate` 翻转。

每改一项重启后复测，直到三轴都符合预期。

---

## 3. Test 2：落点校准（AimOffset）

目的：修正“整体打偏”，不处理运动目标时序问题。

### 3.1 参数

`aimer.toml`：

```toml
[AutoAim.FireControl.AimOffset]
yaw = 0.0    # deg，+ 向左打
pitch = 0.0  # deg，+ 向上打
```

注意：这里是 **deg**，代码内部会自动转弧度。

### 3.2 场地与姿态

1. 放静止装甲板，距离 2~4m。
2. 目标正对己方，避免斜角。
3. 启动自瞄，连续发射 >= 3 发。

### 3.3 调整规则

- 弹着整体偏右：增大 `AimOffset.yaw`
- 弹着整体偏左：减小 `AimOffset.yaw`
- 弹着整体偏下：增大 `AimOffset.pitch`
- 弹着整体偏上：减小 `AimOffset.pitch`

每次建议步进：

- `yaw`: `0.10~0.30 deg`
- `pitch`: `0.10~0.30 deg`

---

## 4. Test 3：延迟调参（运动目标）

目的：修正“跟不上/超前”与“反陀螺打击时机错误”。

---

### 3.1（Test 3.1）调 `additional_predict_time`

参数：

```toml
[AutoAim.FireControl.Cmd]
additional_predict_time = 0.060  # s
```

这是发包前的额外前馈：

- `cmd = aim + additional_predict_time * aim_vel + aim_offset`

操作：

1. 目标做匀速横移。
2. 看 `firecontrol` 视图中黄圈（cmd）与橙圈（aim）相对绿圈（相机光心）的关系。
3. 长期滞后就增大，长期超前就减小。

建议步进：`5~20 ms`。

---

### 3.2（Test 3.2）调 `send_to_control`

参数：

```toml
[AutoAim.FireControl.Latency]
send_to_control = 0.003  # s
```

操作：

1. 匀速平移目标，尽量让跟随先稳定。
2. 发弹观察整体提前/滞后趋势。

规则：

- 总体跟不上：增大 `send_to_control`
- 总体超前：减小 `send_to_control`

建议步进：`0.005~0.020 s`。

---

### 3.3（Test 3.3）调 `control_to_fire`

参数：

```toml
[AutoAim.FireControl.Latency]
control_to_fire = 0.020  # s
```

该参数主要影响反陀螺开火门控时机。

操作：

1. 用稳定陀螺目标（建议 > 20rpm）。
2. 观察“打击瞬间装甲板是否已转过”。

规则：

- 子弹总落在“转过去之后”（慢半拍）：增大 `control_to_fire`
- 子弹总落在“还没转到位”（快半拍）：减小 `control_to_fire`

建议步进：`5~15 ms`。

---

## 5. 火控可视化点位含义（用于定位问题）

`Visualizer.view = "firecontrol"` 下：

- 白圈：图像中心
- 绿圈：相机光心 `(cx, cy)`
- 橙圈：`aim`（弹道解算目标角）
- 黄圈：`cmd`（实际下发角，含额外预测和偏置）
- 红箭头：`aim` 角速度方向（仅可视化）
- 红字 `SHOOT_CMD`：当前 fire 命令触发

判断逻辑：

- 黄圈长期落后橙圈：优先调 `additional_predict_time` / `send_to_control`
- 橙黄整体都偏离目标中心：优先调 `AimOffset`
- 反陀螺“时机错位”明显：调 `control_to_fire`

---

## 6. 弹道参数调法（Trajectory）

当前建议基线（已对齐 rm.cv.fans 线性阻力）：

```toml
[AutoAim.FireControl.Trajectory]
solver = "analytic_linear"
drag_model = "linear"
gravity = 9.8
air_resistance_k = 0.022928514188
max_iter = 10
rk4_dt = 0.001
rk4_max_iter = 50
rk4_tolerance = 0.001
```

说明：

- 常规 2~8m：优先 `analytic_linear`。
- 远距离吊射才切 `rk4_quadratic`，且需要重调 `k`。
- `k` 变大 -> 预测下坠更重（通常会“抬枪更多”）。

---

## 7. 一次完整调参顺序（推荐）

1. **先做 Test 1**：把符号修正对齐（不对齐后面全白调）。
2. **再做 Test 2**：把静态落点打到中心。
3. **最后做 Test 3**：运动目标延迟链路调稳。
4. 如仍有距离相关偏差，再调 `Trajectory.air_resistance_k`。

---

## 8. 常见错误清单

1. `AimOffset` 当弧度填：会一次改太大。  
   现在该参数是 **deg**。
2. `imu_*_negate` 没校准：表现为“怎么调都反着来”。
3. `drag_model` 改了但 `k` 不重调：弹道会明显跑偏。
4. 用反陀螺现象去调 `AimOffset`：会越调越差，应该先调延迟链路。

---

## 9. 相关文档

- 串口协议：[/Users/david/Documents/GitHub/RMCV/docs/serial_protocol_ec_32B.md](/Users/david/Documents/GitHub/RMCV/docs/serial_protocol_ec_32B.md)
- 延迟设计：[/Users/david/Documents/GitHub/RMCV/aimer/auto_aim/fire_control/LATENCY.md](/Users/david/Documents/GitHub/RMCV/aimer/auto_aim/fire_control/LATENCY.md)
