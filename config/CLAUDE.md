# config 目录说明

本目录存放共享配置。两个人协作时，配置文件冲突概率很高，修改前要说明影响范围。

## 常见文件

- `aimer.toml`: 自瞄、火控、可视化等运行时参数。
- `buff.toml`: 能量机关运行时参数。
- `armor_detector.toml`: 装甲板检测器配置。
- `buff_detector.toml`: 能量机关检测器配置。
- `hardware.toml`: 相机、串口、fake serial 等硬件配置。
- `debugger.toml`: 调试相关配置。
- `camera.yaml`: 相机标定和外参，供 transformer 初始化。

## TOML 类型规则

- 需要 `double` 时写 `2.0`，不要写 `2`。
- 需要 `int64_t` 时写 `2`。
- 需要 `bool` 时写 `true` / `false`。
- 字符串使用双引号。

## 修改规则

- 新增参数时同步更新对应模块的读取代码和注释。
- 改名或移动参数路径时，全仓库搜索旧路径。
- 不要把个人机器路径、临时端口、一次性调参结果直接提交到共享配置。
- 机器本地差异建议使用未提交的 local 配置或启动参数解决。

## 验证

```bash
./build/test_param
```
