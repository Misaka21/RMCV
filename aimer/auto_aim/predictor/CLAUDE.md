# auto_aim predictor 说明

本目录负责装甲板跨帧跟踪、敌方车辆状态估计和短期预测。

## 入口与输出

- 入口线程：`predictor_node.cpp`
- 主类：`EnemyPredictor`
- 输入：`Message<DetectionResult>("detections")`
- 输出：`BasicObjManager<BattlefieldSnapshot>("battlefield")`
- 调试状态：`BasicObjManager<PredictorDebugFrame>("predictor_debug")`

## 设计目标

predictor 输出的是火控可直接读取的战场快照。火控会以更高频率读取同一份快照，
并根据时间戳和速度做短期插值。

```text
DetectionResult
    -> observer/tracker
    -> enemy model
    -> BattlefieldSnapshot
    -> fire_control
```

## 数据结构约定

- `BattlefieldSnapshot.timestamp` 使用相机帧时间，单位秒。
- `BattlefieldSnapshot.predict_timestamp` 使用预测完成时间，供火控估算
  `img_to_predict` 延迟。
- `BattlefieldSnapshot.self_state` 保存检测时刻自身状态。
- `VehicleState` 表示单个目标整体状态。
- `ArmorState` 表示单块装甲板位置、速度、朝向、可见性和评分。

## 模型边界

- `observer/`: 从检测结果建立观测、维护装甲板表和 tracker。
- `model/`: 敌方运动模型，不直接处理图像。
- predictor 可以输出推荐目标或评分，但最终射击决策归 fire_control。

## 运行时参数

EKF 噪声、阈值和开关必须在使用点调用
`runtime_param::get_param<T>()`。不要把参数封装进只加载一次的 config struct。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_fire_control
```
