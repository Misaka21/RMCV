# auto_aim fire_control 说明

本目录负责自瞄火控：目标选择、延迟补偿、弹道解算、射击决策和命令输出。

## 入口与输出

- 入口线程：`fire_control_node.cpp`
- 控制器：`fire_controller.*`
- 目标选择：`selection/`
- 开火决策：`decision/`
- 输入：`BasicObjManager<BattlefieldSnapshot>("battlefield")`
- 输出：`BasicObjManager<::fire_control::FireCommand>("fire_command")`

## 频率与时间

火控循环目标频率是 500Hz。不要在主循环里加入重型图像处理、模型推理或阻塞 IO。

延迟补偿使用：

- `snapshot.timestamp`: 图像采集时间。
- `snapshot.predict_timestamp`: predictor 完成时间。
- 当前 steady clock 时间。
- 弹速和目标距离。

## 边界规则

- fire_control 不订阅原始图像，不重新检测。
- fire_control 只消费 predictor 快照和自身状态。
- 射击决策必须同时考虑目标可用性、模式、快照是否过期、allow_fire 和弹道解。
- 输出给串口的角度单位必须与 `hardware/serial` 协议一致。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_fire_control
./build/test_ballistic
```
