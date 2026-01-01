# RMCV Scripts

综合 FYT2024 和 sp_vision_25 优点的启动脚本。

## 快速开始

```bash
# 安装服务 (开机自启 = 比赛模式)
cd scripts
sudo ./install_service.sh

# 手动调试 (不录制)
./watchdog.sh
```

---

## 比赛模式 vs 调试模式

| 模式 | 启动方式 | 录制 | 使用场景 |
|------|----------|------|----------|
| **比赛模式** | `./watchdog.sh --match` | 强制录 raw.mkv + imu.csv | 正式比赛、需要回放分析 |
| **调试模式** | `./watchdog.sh` | 按 recorder.toml 配置 | 日常调试、测试 |

### 什么时候用比赛模式？

- ✅ **正式比赛** - 开机自启默认比赛模式，强制内录
- ✅ **需要录像回放** - 即使 recorder.toml 关闭录制，也会强制录
- ✅ **赛后分析** - 确保有数据可以复盘

### 什么时候用调试模式？

- ✅ **日常调试** - 不需要录制，节省磁盘空间
- ✅ **参数调整** - 快速测试，不产生大量视频文件
- ✅ **开发测试** - 灵活控制录制开关

---

## 配置指南

### 1. 开机自启配置 (rmcv.service)

开机自启 = systemd 启动 = **默认比赛模式**

```bash
# 修改 rmcv.service 第 25 行:
ExecStart=/home/user/RMCV/scripts/watchdog.sh --match   # 比赛模式 (默认)
ExecStart=/home/user/RMCV/scripts/watchdog.sh           # 调试模式
```

安装后修改:
```bash
sudo systemctl edit rmcv --force
# 添加:
# [Service]
# ExecStart=
# ExecStart=/path/to/watchdog.sh    # 不带 --match
sudo systemctl daemon-reload
```

### 2. 看门狗配置 (watchdog.sh 头部)

```bash
# ========== 必须配置 ==========
TIMEOUT=10              # 心跳超时 (秒)
                        # - 太短: 正常启动可能被误判超时
                        # - 太长: 卡死检测不及时
                        # - 建议: 5-15 秒

MAX_RETRY=100           # 最大重启次数
                        # - 超过此数则停止尝试
                        # - 比赛时建议: 100+
                        # - 调试时可以: 10

# ========== 进程优先级 (可选) ==========
ENABLE_REALTIME=true    # 实时调度
                        # - true: 更低延迟，需要 root 或配置 limits
                        # - false: 普通调度

NICE_LEVEL=-15          # Nice 优先级 (-20 ~ 19)
                        # - -20: 最高优先级
                        # - 0: 默认
                        # - 建议: -10 ~ -15

RT_PRIORITY=50          # 实时调度优先级 (1-99)
                        # - 越高越优先
                        # - 不要超过 80 (留给系统关键进程)
                        # - 建议: 40-60

# CPU_AFFINITY="0-3"    # CPU 绑定 (取消注释启用)
                        # - 绑定到特定核心，提高缓存命中
                        # - 格式: "0-3" 或 "0,2,4"
                        # - NUC 通常 4-8 核，可绑定一半
```

### 3. 录制配置 (config/recorder.toml)

```toml
[Recorder]
enable_recording = false    # 调试模式下是否录制
                            # 比赛模式会忽略此项，强制录制

record_raw_video = true     # 录制原始画面
record_debug_video = true   # 录制调试画面 (比赛模式也按此配置)
record_imu_csv = true       # 录制 IMU 数据

camera_fps = 200.0          # 相机帧率
sample_interval = 3         # 采样间隔 (3 = 每3帧录1帧 = 66fps)
video_codec = "MJPG"        # 编码格式
```

**比赛模式强制录制规则:**
- `raw.mkv` - 强制录制 ✅
- `imu.csv` - 强制录制 ✅
- `debug.mkv` - 按配置决定 (不强制)

---

## 推荐配置

### 比赛场景
```bash
# watchdog.sh
TIMEOUT=10
MAX_RETRY=100
ENABLE_REALTIME=true
NICE_LEVEL=-15
RT_PRIORITY=50
```

```toml
# recorder.toml
enable_recording = false   # 会被 --match 覆盖
record_raw_video = true
record_debug_video = false # 比赛时关闭，节省性能
record_imu_csv = true
sample_interval = 3        # 66fps 足够回放
```

### 调试场景
```bash
# watchdog.sh
TIMEOUT=15          # 给更多启动时间
MAX_RETRY=10        # 快速失败
ENABLE_REALTIME=false
```

```toml
# recorder.toml
enable_recording = false   # 不录制
```

### 录像分析场景
```bash
./watchdog.sh --match   # 手动启用比赛模式
```

---

## 文件说明

| 文件 | 功能 |
|------|------|
| `watchdog.sh` | 看门狗 (screen + 心跳 + 优先级) |
| `cleanup.sh` | 停止所有进程 |
| `install_service.sh` | 安装 systemd 服务 |
| `rmcv.service` | systemd 配置模板 |

## 常用命令

```bash
# systemd 管理
sudo systemctl start rmcv     # 启动
sudo systemctl stop rmcv      # 停止
sudo systemctl restart rmcv   # 重启
sudo systemctl status rmcv    # 状态
journalctl -u rmcv -f         # 日志

# 手动运行
./watchdog.sh --match         # 比赛模式
./watchdog.sh                 # 调试模式
./cleanup.sh                  # 停止所有

# 调试
screen -r rmcv                # 连接终端
screen -d rmcv                # 断开 (不关闭)
Ctrl+A, D                     # screen 内断开
```

## 日志位置

```
log/
├── watchdog_20240101_120000.log   # 看门狗日志
├── latest -> 20240101_120000/     # 最新会话链接
└── 20240101_120000/               # 会话目录
    ├── console.log                # RMCV 输出
    ├── heartbeat                  # 心跳文件
    ├── raw.mkv                    # 原始视频
    ├── debug.mkv                  # 调试视频
    └── imu.csv                    # IMU 数据
```

## 架构图

```
开机
  │
  └─> systemd (rmcv.service)
         │
         └─> watchdog.sh --match
                │
                ├─ 启动 screen 会话
                │     └─> RMCV2026 --match
                │            │
                │            ├─ 检测 --match 参数
                │            ├─ 设置 match_mode = true
                │            └─ 强制录制 raw + imu
                │
                ├─ 设置进程优先级
                │
                └─ 健康检查循环 (每 TIMEOUT 秒)
                      ├─ screen 存活?
                      ├─ 进程存活?
                      └─ 心跳更新?
                            │
                            └─> 失败则 restart
```
