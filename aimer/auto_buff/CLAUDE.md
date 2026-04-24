# auto_buff 模块说明

`aimer/auto_buff/` 是能量机关业务链路，结构与自瞄类似，但目标、预测模型和合作策略
独立维护。

## 子模块职责

- `common/`: 能量机关共享类型。
- `detector/`: 能量机关检测、预处理、后处理、解码。
- `predictor/`: 方向估计、模式管理、小符/大符模型和槽位消抖。
- `fire_control/`: 能量机关火控、目标排序和合作策略。

## 数据流

```text
SyncFrame -> buff detector -> buff predictor -> buff fire_control -> FireCommand
```

## 改动原则

- 不要让 auto_buff 依赖 auto_aim 内部 detector/predictor/fire_control 实现。
- 可共享的数学、坐标、弹道、延迟能力放到 `aimer/common`。
- 小符和大符模型要保持接口一致，模式切换逻辑放在 predictor 层。
- 火控合作策略放在 `fire_control/coop_policy.*`，不要散落到 detector 或 predictor。

## 验证

```bash
cmake --build build -j$(nproc)
```
