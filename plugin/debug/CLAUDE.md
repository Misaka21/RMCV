# debug 模块说明

本目录负责日志输出和 session 日志目录管理。

## 改动规则

- 日志模块不能依赖业务层。
- 高频循环中的日志必须限频或只在状态变化时输出。
- 面向人的解释可以用中文；日志 key、模块名和调试通道名优先英文。
- 初始化逻辑在主程序早期执行，修改时注意 watchdog 模式传入的 `--log-dir`。

## 验证

```bash
cmake --build build -j$(nproc)
./build/RMCV2026 --help
```
