# simulator 模块说明

本目录是可选仿真节点。主 CMake 会添加该目录，但实际功能可能依赖 ROS2 或仿真环境。

## 改动规则

- 保持没有仿真环境的机器仍能正常构建主项目。
- 仿真输入输出要尽量贴近硬件层语义，方便复用 detector、predictor 和 fire_control。
- 不要让主业务代码强依赖 simulator。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_simulator
```
