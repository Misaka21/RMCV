# RMCV2026

RoboMaster Computer Vision - 机器人视觉系统

## 项目简介

RMCV 是一个用于 RoboMaster 机器人的视觉识别与自动瞄准系统，采用 C++17 开发。

## 项目结构

```
RMCV2026/
├── aimer/                      # 自动瞄准系统
│   ├── common/                 # 公共模块
│   │   ├── math/               # 数学工具 (坐标转换等)
│   │   └── transformer/        # TF坐标变换系统
│   └── auto_aim/               # 自瞄核心
│       └── detector/           # 目标检测
│           ├── detector_rv/    # 传统视觉检测器
│           └── detector_yolo/  # YOLO检测器 (可选)
│
├── hardware/                   # 硬件抽象层
│   ├── hik_cam/                # 海康相机驱动
│   └── serial/                 # 串口通信
│
├── plugin/                     # 插件系统
│   ├── debug/                  # 日志系统
│   └── param/                  # 参数管理 (静态/动态)
│
├── umt/                        # 线程间通信框架
│   ├── Message.hpp             # 发布-订阅消息
│   ├── ObjManager.hpp          # 类对象管理
│   └── BasicObjManager.hpp     # 基础类型管理
│
├── config/                     # 配置文件
│   ├── camera.yaml             # 相机标定参数 (静态)
│   ├── aimer.toml              # 瞄准参数 (动态热更新)
│   └── hardware.toml           # 硬件配置
│
└── test/                       # 测试程序
```

## 快速开始

### 依赖安装

```bash
# Ubuntu/Debian
sudo apt install cmake libopencv-dev libfmt-dev libeigen3-dev

# tomlplusplus (header-only)
git clone https://github.com/marzer/tomlplusplus.git
sudo cp -r tomlplusplus/include/toml++ /usr/local/include/

# pybind11
sudo apt install pybind11-dev

# HIK MVS SDK - 从海康官网下载安装
```

### 编译

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

### 运行

```bash
./RMCV2026              # 普通运行
./RMCV2026 --web        # 带 Web 调试界面 (http://localhost:5000)
./RMCV2026 --match      # 比赛模式 (强制内录)
```

### 服务管理

安装 systemd 服务 (开机自启)：
```bash
sudo ./scripts/install_service.sh
```

服务控制命令：
```bash
sudo systemctl start rmcv     # 启动
sudo systemctl stop rmcv      # 停止
sudo systemctl status rmcv    # 查看状态
sudo systemctl restart rmcv   # 重启

# 开机自启控制
sudo systemctl enable rmcv    # 启用开机自启
sudo systemctl disable rmcv   # 禁用开机自启

# 查看日志
journalctl -u rmcv -f         # 实时日志
screen -r rmcv                # 连接终端 (Ctrl+A D 断开)
```

### 调试模式

调试前需要先停止服务：
```bash
# 停止 + 禁用开机自启
sudo systemctl disable --now rmcv

# 手动运行调试
cd build
./RMCV2026 --web

# 调试完成后恢复
sudo systemctl enable --now rmcv
```

或使用清理脚本：
```bash
./scripts/cleanup.sh          # 停止所有 RMCV 相关进程
```

### 测试

```bash
./test_transformer   # 坐标变换测试
./test_param         # 参数系统测试
./test_camera        # 相机测试
./test_serial        # 串口测试
```

## 核心模块

### 坐标变换系统 (TF)

编译期路径推导 + 运行期动态参数的坐标变换系统。

```cpp
#include "aimer/common/transformer/transformer.hpp"

// 初始化
tf::init();

// Camera -> World 变换
Eigen::Vector3d p_world = tf::cam_to_world(p_cam, q_imu);

// 里程计积分
tf::update_odometry(Eigen::Vector3d(vx, vy, 0), dt, q_imu);
```

**坐标系定义：**

| 坐标系 | 说明 | 轴向 |
|--------|------|------|
| World | 上电时云台位置 | X右 Y下 Z前 |
| Imu | IMU芯片坐标系 | 硬件定义 |
| Gimbal | 云台坐标系 | X右 Y下 Z前 |
| Camera | 相机坐标系 | X右 Y下 Z前 |
| Barrel | 枪口坐标系 | X右 Y下 Z前 |

详细文档：[aimer/common/transformer/README.md](aimer/common/transformer/README.md)

### 参数系统

**静态参数** (YAML, 启动时加载)：
```cpp
tf::init();  // 加载 config/camera.yaml
```

**动态参数** (TOML, 运行时热更新)：
```cpp
runtime_param::parameter_run("aimer.toml");
double value = runtime_param::get_param<double>("section.key");
```

### UMT 线程通信

```cpp
// 发布-订阅
umt::Publisher<MyData> pub("channel");
umt::Subscriber<MyData> sub("channel");
pub.push(data);
auto msg = sub.pop();

// 共享对象
auto obj = umt::BasicObjManager<float>::find_or_create("threshold", 0.5f);
obj->get() = 0.8f;
```

## 配置文件

### camera.yaml (静态标定)

```yaml
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # IMU安装修正
R_camera2gimbal: [1, 0, 0, 0, 1, 0, 0, 0, 1]   # 相机安装角度
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1] # 相机内参
distort_coeffs: [k1, k2, p1, p2, k3]           # 畸变系数
```

### aimer.toml (动态调参)

```toml
[Transformer]
camera_offset_x = 0.0   # 相机偏移 (米)
camera_offset_y = 0.0
camera_offset_z = 0.0
barrel_offset_x = 0.0   # 枪口偏移 (米)
barrel_offset_y = 0.055
barrel_offset_z = 0.0
```

## 开发规范

### 代码风格

- 使用 `.clang-format` 格式化代码
- 命名规范：类 `CamelCase`，函数/变量 `snake_case`
- 注释使用中文，日志输出使用英文

### Git 提交

```
✨ feat: 新功能
🐛 fix: 修复bug
📝 docs: 文档更新
♻️ refactor: 重构
✅ test: 测试
🔧 chore: 配置/构建
```

## 实用命令

### 整理录制文件夹

将 `YYYY-MM-DD_HH-MM-SS` 格式的录制文件夹按日期归类：

```bash
for dir in [0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]_*; do date="${dir:0:10}"; mkdir -p "$date"; mv "$dir" "$date/"; done
```

## 许可证

MIT License
