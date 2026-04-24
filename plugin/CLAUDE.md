# plugin 模块说明

`plugin/` 放基础设施，不放业务策略。这里的代码会被硬件、自瞄、能量机关和测试共同使用。

## 子模块

- `debug/`: 日志系统。
- `param/`: 静态配置和运行时参数热重载。
- `stats/`: FPS 和延迟统计。
- `visualizer/`: 本地可视化窗口。
- `rmcv_bag/`: 录制与回放。
- `rerun/`: Rerun 可视化适配。
- `watchdog/`: 线程心跳和进程健康检查。

## 改动规则

- plugin 不能依赖 auto_aim 或 auto_buff 内部实现。
- 公共工具要保持接口小而稳定。
- 高频路径中的日志和可视化要有开关，避免污染实时性能。
- 参数系统改动必须同步检查 `config/` 和 `test_param`。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_param
./build/test_playback
```
