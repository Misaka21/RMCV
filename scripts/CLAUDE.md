# scripts 目录说明

本目录存放部署、服务、看门狗和辅助脚本。

## 文件说明

- `install_service.sh`: 安装 systemd 服务。
- `rmcv.service`: systemd 服务文件。
- `watchdog.sh`: 进程级看门狗脚本。
- `cleanup.sh`: 清理 RMCV 相关进程。
- `add_preprocess_to_onnx.py`: 模型预处理辅助。

## 改动规则

- 修改服务脚本时注意比赛机器的实际路径和权限。
- 需要 sudo 的脚本要保持显式，不要在普通构建或测试中隐式执行。
- shell 脚本要能重复执行，避免残留半安装状态。
- 改 systemd 行为时同步更新 `scripts/README.md`。

## 常用命令

```bash
sudo ./scripts/install_service.sh
./scripts/cleanup.sh
```
