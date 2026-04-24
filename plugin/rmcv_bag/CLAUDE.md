# rmcv_bag 模块说明

本目录负责录制和回放。比赛模式会强制内录，相关逻辑需要保持稳定。

## 改动规则

- 录制和回放格式变更要考虑向后兼容或写清迁移方式。
- 不要让 recorder 阻塞主业务线程。
- 回放数据应尽量复现 `hardware::SyncFrame` 的语义，方便 detector/predictor 离线验证。
- 修改比赛模式行为时同步检查 `main.cpp` 中的 `match_mode`。

## 验证

```bash
cmake --build build -j$(nproc)
./build/test_playback
```
