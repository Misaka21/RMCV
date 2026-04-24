# auto_aim 模块说明

`aimer/auto_aim/` 是装甲板自瞄业务链路。核心目标是把同步帧转换为火控命令：

```text
SyncFrame -> DetectionResult -> BattlefieldSnapshot -> FireCommand
```

## 子模块职责

- `detector/`: 从图像中识别装甲板，输出 `DetectionResult`。
- `predictor/`: 跨帧跟踪和预测敌方车辆状态，输出 `BattlefieldSnapshot`。
- `fire_control/`: 选目标、延迟补偿、弹道解算、射击决策，输出 `FireCommand`。
- `common/`: 自瞄内部共享类型。

## 线程与通道

- detector 订阅 `"sync_frame"`，发布 `"detections"`。
- predictor 订阅 `"detections"`，写入 `BasicObjManager<BattlefieldSnapshot>("battlefield")`。
- fire_control 高频读取 `"battlefield"`，写入 `BasicObjManager<FireCommand>("fire_command")`。

## 改动原则

- detector 不做长期状态预测。
- predictor 不直接决定是否开火。
- fire_control 不重新做图像识别或长期跟踪。
- 新增跨阶段字段时，优先扩展明确的数据结构，不通过散乱的全局对象传递。
- 参数读取遵守根目录运行时参数硬规则，不缓存热重载参数。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_fire_control
./build/test_ballistic
```
