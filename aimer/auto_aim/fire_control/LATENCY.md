# 延迟补偿设计文档

## 1. 延迟链路模型

视觉自瞄系统从拍摄到命中，经历 6 个时间节点：

```
img → predict → send → control → fire → hit
 │       │        │        │        │      │
 t0      t1       t2       t3       t4     t5
```

| 阶段 | 符号 | 含义 | 来源 |
|-----|------|------|------|
| img → predict | `img_to_predict` | 图像采集 + 推理 + 预测计算 | 直接测量 |
| predict → send | `predict_to_send` | 准备发送 + 线程调度 | 卡尔曼滤波 |
| send → control | `send_to_control` | 串口传输 + 下位机解析 | 静态配置 |
| control → fire | `control_to_fire` | 电机响应 + 机械传动 | 静态配置 |
| fire → hit | `fire_to_hit` | 子弹飞行时间 | **迭代计算** |

### 1.1 fire_to_hit 的迭代计算

`fire_to_hit` 由 **TrajectorySolver** 计算的 `fly_time` 提供，已包含空气阻力等因素。

**问题**：弹道解算需要预测位置 → 预测位置需要 `prediction_latency()` → `prediction_latency()` 需要 `fire_to_hit` → 鸡生蛋

**解决**：迭代收敛（参考 rm.cv.fans `filter_to_prediction_time`）

```cpp
// fire_controller.cpp
constexpr int NUM_ITERATIONS = 2;
for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
    // 用当前延迟预测位置
    double dt = latency.prediction_latency();
    Eigen::Vector3d predicted_pos = target.predict_position(dt);

    // 弹道解算 (含空气阻力)
    AimResult aim = trajectory_solver.solve(predicted_pos, bullet_speed);

    // 用弹道解算的飞行时间更新 fire_to_hit
    latency.set_fly_time(aim.fly_time);
}
```

**收敛性**：通常 2 次迭代即可收敛，因为：
- 初始估计（装甲板距离）与最终值（瞄准点距离）相差很小（几厘米）
- 飞行时间变化导致的位置变化更小（二阶小量）

## 2. 两个延迟量的哲学

### 2.1 预测延迟 (prediction_latency)

**定义**：
```cpp
prediction_latency = img_to_predict + predict_to_send + send_to_control + fire_to_hit
```

**不含** `control_to_fire`！

**为什么？** 参考 rm.cv.fans 的设计：

- 我们预测的是**目标在控制生效时刻的位置**
- 控制生效后，云台开始运动追踪该位置
- 子弹出膛需要额外的 `control_to_fire` 时间，但此时**云台也在运动**
- 云台的运动和子弹飞行是**并行**的，相互抵消

**直观理解**：
```
t3 (控制生效) ─────────────────────→ t5 (命中)
      │                                  │
      │  云台从 θ_current → θ_target     │
      │  子弹从 枪口 → 目标              │
      │                                  │
      └──── 两者同时进行，延迟不叠加 ────┘
```

### 2.2 命中延迟 (hit_latency)

**定义**：
```cpp
hit_latency = img_to_predict + predict_to_send + send_to_control + control_to_fire + fire_to_hit
```

**用途**：反陀螺开火时机判断

反陀螺模式下，我们需要知道：从当前时刻 t0 拍摄的这帧图像，到子弹实际命中目标，总共需要多少时间。这决定了我们预测的目标相位。

## 3. 参数详解与调参方法

配置位于 `config/aimer.toml` 的 `[AutoAim.FireControl.Latency]` 部分。

### 3.1 img_to_predict (默认 10ms)

**含义**：图像采集 + 神经网络推理 + PnP解算 + EKF预测

**测量方法**：
```cpp
// predictor_node.cpp 中已经计算
img_to_predict = predict_timestamp - snapshot.timestamp
```

**调参**：
- 通常不需要手动设置，代码会自动计算
- 默认值 10ms 作为 fallback

### 3.2 predict_to_send (默认 2ms)

**含义**：预测完成到火控发送的时间

**特点**：使用卡尔曼滤波平滑

**调参**：
- 正常情况下自动估计，无需调整
- 如果观察到抖动，可以增大滤波器的 R 值 (观测噪声)

### 3.3 send_to_control (默认 3ms) ⭐ 重要

**含义**：串口传输 + 下位机解析 + 控制器响应延迟

**调参方法**：

**实验 1: 跟踪匀速运动目标**
1. 让目标以恒定速度水平移动
2. 观察云台跟踪是否存在**稳定的角度滞后**
3. 如果滞后，增大 `send_to_control`；如果超前，减小

**实验 2: 静态目标跟踪**
1. 目标静止
2. 如果云台有明显抖动/过冲，可能是延迟估计过大

**参数方向**：
- `+` 调大：跟踪更超前，适合延迟大的系统
- `-` 调小：跟踪更保守

### 3.4 control_to_fire (默认 20ms) ⭐ 反陀螺关键

**含义**：从云台收到指令到子弹出膛的时间

**调参方法**：

**实验: 反陀螺打击**
1. 目标稳定陀螺（已知角速度）
2. 观察命中位置：
   - 打在装甲板**后方** (目标旋转方向的后面) → 延迟估计**偏小**，需要调大
   - 打在装甲板**前方** (目标旋转方向的前面) → 延迟估计**偏大**，需要调小

**参数方向**：
- `+` 调大：提前打击，子弹在装甲板转到位之前出膛
- `-` 调小：延后打击

### 3.5 additional_predict_time (默认 60ms)

**含义**：在最终下发命令阶段增加一段“额外前瞻”：
```
cmd_yaw   = plan_yaw   + additional_predict_time * plan_yaw_vel
cmd_pitch = plan_pitch + additional_predict_time * plan_pitch_vel
```

**与 `send_to_control` 的区别**：
- `send_to_control` 属于时间链路测量值，影响 `prediction_dt`
- `additional_predict_time` 是发包前补偿，主要用于电控跟随滞后微调

**调参方法**：
1. 让目标做匀速平移
2. 观察长期跟随偏差
3. 若总体滞后，增大 `additional_predict_time`
4. 若总体超前，减小 `additional_predict_time`

**安全项**：
- `max_abs_vel` 用于限制参与额外前瞻的角速度，避免切板瞬间尖峰

## 4. 调参优先级

1. **先调 send_to_control**：影响所有跟踪精度
2. **再调 control_to_fire**：只影响反陀螺
3. **最后调 additional_predict_time**：修正长期跟随滞后/超前

## 5. 典型配置参考

### 高速响应系统 (响应快的电控)
```toml
[AutoAim.FireControl.Latency]
send_to_control = 0.002       # 2ms
control_to_fire = 0.015       # 15ms
[AutoAim.FireControl.Cmd]
additional_predict_time = 0.040
max_abs_vel = 12.0
```

### 普通响应系统
```toml
[AutoAim.FireControl.Latency]
send_to_control = 0.003       # 3ms
control_to_fire = 0.020       # 20ms
[AutoAim.FireControl.Cmd]
additional_predict_time = 0.060
max_abs_vel = 12.0
```

### 高延迟系统 (响应慢的电控)
```toml
[AutoAim.FireControl.Latency]
send_to_control = 0.005       # 5ms
control_to_fire = 0.030       # 30ms
[AutoAim.FireControl.Cmd]
additional_predict_time = 0.080
max_abs_vel = 12.0
```

## 6. 代码实现

### 延迟信息结构
```cpp
// types.hpp
struct LatencyInfo {
    double img_to_predict = 0;     // 图像→预测完成
    double predict_to_send = 0;    // 预测→发送
    double send_to_control = 0;    // 发送→控制器响应
    double control_to_fire = 0;    // 控制器→出膛
    double fire_to_hit = 0;        // 出膛→命中 (TrajectorySolver fly_time)
    double bullet_speed = 15.0;    // 弹速 (用于 latency_estimator 构建初始值)

    // 设置 fire_to_hit (使用弹道解算器计算的飞行时间)
    void set_fly_time(double fly_time);

    // 预测延迟 (不含 control_to_fire)
    double prediction_latency() const;

    // 命中延迟 (含 control_to_fire)
    double hit_latency() const;

    // 从当前时刻到命中
    double now_to_hit() const;
};
```

### 延迟构建 (LatencyEstimator)
```cpp
// latency_estimator.hpp
LatencyInfo build(const BattlefieldSnapshot& snapshot, double current_time) const {
    LatencyInfo latency;

    // 自动计算
    latency.img_to_predict = snapshot.predict_timestamp - snapshot.timestamp;
    latency.predict_to_send = get_predict_to_send();  // 卡尔曼滤波

    // 静态配置
    latency.send_to_control = get_param("send_to_control");
    latency.control_to_fire = get_param("control_to_fire");

    // 初始估计 (后续迭代更新)
    latency.bullet_speed = snapshot.self_state.bullet_speed;
    latency.fire_to_hit = armor_distance / latency.bullet_speed;

    return latency;
}
```

### 火控迭代
```cpp
// fire_controller.cpp
FireCommand FireController::control(...) {
    LatencyInfo latency = latency_in;  // 复制

    // 目标选择 (用初始延迟)
    TargetSelection selection = target_selector_.select(snapshot, latency.prediction_latency());

    // 迭代更新 fire_to_hit
    for (int i = 0; i < 2; ++i) {
        auto predicted_pos = target.predict(latency.prediction_latency());
        auto aim = trajectory_solver_.solve(predicted_pos, bullet_speed);
        latency.set_fly_time(aim.fly_time);  // 使用弹道解算的飞行时间
    }

    // 后续处理用更新后的延迟
    ...
}
```

### 使用场景
```cpp
// 1. 目标位置预测
double dt = latency.prediction_latency();
Eigen::Vector3d predicted_pos = armor.position + armor.velocity * dt;

// 2. 反陀螺相位预测
double dt_hit = latency.hit_latency();
double predicted_phase = spin.phase + spin.omega * dt_hit;

// 3. 开火时机判断
double time_to_hit = latency.now_to_hit();
if (time_to_fire <= time_to_hit) {
    fire_now = true;
}
```

## 7. 常见问题

### Q1: 为什么不把所有延迟加起来预测？

A: 因为 `control_to_fire` 期间云台也在运动。如果全加上，会**预测过度**，导致跟踪超前。

### Q2: 子弹飞行时间 fire_to_hit 应该算在哪里？

A: 两个地方都要算：
- `prediction_latency()` 里包含，因为需要预测子弹到达时目标的位置
- `hit_latency()` 里也包含，因为反陀螺需要知道完整的命中时刻

### Q3: 如何验证延迟估计是否正确？

A: 三个验证实验：
1. **静态目标**：云台应该稳定指向，无抖动
2. **匀速目标**：云台应该无稳态误差地跟踪
3. **陀螺目标**：命中点应该在装甲板中心

### Q4: 卡尔曼滤波 predict_to_send 的意义？

A: 线程调度导致 `predict_to_send` 有波动，直接使用会引入抖动。滤波后更平滑，减少云台抖动。

### Q5: 为什么 fire_to_hit 需要迭代计算？

A: 因为 `fire_to_hit` 依赖弹道解算后的距离，而弹道解算需要预测位置，预测位置又需要 `fire_to_hit`。

**rm.cv.fans 的解法**：迭代 2-3 次收敛
```cpp
// coord_converter.cpp:562-571
double CoordConverter::filter_to_prediction_time(const PositionPredictorInterface& filter) const {
    int iterations_num = base::get_param<int64_t>("auto-aim.predict.num-iterations");
    double prediction_t = this->get_img_t();
    for (int i = 0; i < iterations_num; ++i) {
        prediction_t = this->get_prediction_time(
            this->target_pos_to_shoot_param(filter.predict_pos(prediction_t)).aim_xyz_i_barrel
        );
    }
    return prediction_t;
}
```

**为什么能快速收敛**：
- 初始估计（装甲板原始距离）和最终值（瞄准点距离）相差很小（几厘米）
- 飞行时间变化 → 预测位置变化 → 新飞行时间变化，这是二阶小量
- 2 次迭代误差已经小于 0.1ms
