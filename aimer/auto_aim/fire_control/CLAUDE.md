# auto_aim fire_control 说明

本目录负责自瞄火控：目标选择、延迟补偿、弹道解算、射击决策和命令输出。

## 入口与输出

- 入口线程：`fire_control_node.cpp`
- 控制器：`fire_controller.*`
- 输入：`BasicObjManager<BattlefieldSnapshot>("battlefield")`
- 输出：`BasicObjManager<::fire_control::FireCommand>("fire_command")`

## 目录结构

```
fire_control/
├── types.hpp              # 模块类型 (TargetSelection, AimMode, ArmorAimResult, FireGateDebug)
├── fire_controller.hpp    # 火控主类 — 算法驱动，对齐 rm.cv.fans
├── fire_controller.cpp    # 实现
├── fire_control_node.hpp  # 节点入口
├── fire_control_node.cpp  # 500Hz 主循环 + 调试输出
└── planner/               # 云台规划 (MPC/PID)
```

火控逻辑全部内聚在 `FireController` 中，不再拆分为 selection/、decision/ 子目录。

## 核心算法 (对齐 rm.cv.fans)

1. **选敌**: TargetCatcher（catch → hold → timeout 三段式）
2. **装甲板瞄准**:
   - 陀螺：direct（窗口内 swing_cost 最小）→ indirect（最早入窗，守株待兔）
   - 非陀螺：可见板中选最正对的
3. **延迟迭代**: 迭代收敛 fire_to_hit 与选板
4. **开火门控**: tracking + swing + out + rotate_back
5. **指令生成**: 位置 + 速度前馈 + aim offset

## 频率与时间

火控循环目标频率是 500Hz。不在主循环里做重型图像处理、模型推理或阻塞 IO。

## 边界规则

- fire_control 不订阅原始图像，不重新检测。
- fire_control 只消费 predictor 快照和自身状态。
- 射击决策同时考虑目标可用性、模式、快照是否过期、allow_fire 和弹道解。
- 输出给串口的角度单位与 `hardware/serial` 协议一致。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_ballistic
```
