# 1.13
移植 sp_vision_25 的 SP 整车运动模型。原 SpinMotion 用两个半径 r1、r2 表示四装甲板分布，切板时交换 r1 ↔ r2 导致状态量突变，协方差无法正确传递。SP 模型改用差量 L = r1 - r2、H = z1 - z2，切板只需 θ += π，状态连续。11维状态向量：[xc, vx, yc, vy, zc, vz, θ, ω, r, L, H]。配置 motion_model 可选 spin/lmtd/sp 三种模型。
给AdaptiveEkf 新增马氏距离门限检查。计算 d² = (z - Hx̂)ᵀ S⁻¹ (z - Hx̂)，超过 χ² 门限判定为离群点，拒绝更新并增大过程噪声 Q 进入宽松模式。连续拒绝超过阈值则从当前观测重新初始化。χ² 门限取 p=0.01，3维观测 11.34，4维观 测 13.28。SpinMotion、ArmorMotion、OutpostMotion 均已适配。
解决sp_motion Gating 失效问题。自适应观测噪声 R 使用观测值计算，当观测本身为离群点时 R 偏大，任何观测都能通过 门限。改为使用预测位置计算 R。
SpinMotion 跳变检测坐标系不匹配。检测逻辑位于 vehicle_model 外部，使用观测坐标系角度，与 EKF 内部状态坐标 系存在 θ 偏移。将跳变检测移至 SpinMotion 内部，直接使用状态量比较。
LmtdMotion 切板时交换 r1、r2 后 predict_armor_position() 返回值跳变，原因是 recommended_armor_idx 未同步更新。
Predictor 与 FireControl 数据共享问题。Predictor 使用 Publisher 发布 BattlefieldSnapshot，FireControl 使用 BasicObjManager 读取，两者机制不同导致数据不通。统一改为 BasicObjManager。
火控可视化改为准心 + 圆环模型。准心固定于图像中心表示枪口指向，圆环位置为弹道解算后的瞄准方向，圆环半径对 应开火容许误差。准心落入圆环内即表示可开火。
Motion 工厂抽象重构。vehicle_model.cpp 存在 20+ 处 if-else 分支判断 use_sp/use_lmtd，每次调用 predict_center()、get_velocity() 都要三选一，添加新模型要改十几处。新增 MotionInterface 抽象基类定义统一接口，工厂函数 create_motion(type, armor_num) 根据配置创建模型。VehicleModel 改用 unique_ptr 单指针替代三个成员变量，ensure_motion_model() 实现延迟初始化和运行时热切换。添加新模型只需实现接口 + 注册工厂 。
完善接口设计细节。get_velocity() 统一返回中心速度，LmtdMotion 原有 get_center_velocity() 保留为内部实现。compute_all_armors(dt) 替代原 compute_all_armors_from_observation()，直接从 EKF 状态生成装甲板位置不依赖外部观测 。SpinState 相位计算修正：原代码 LmtdMotion 的 theta 是当前追踪装甲板角度，需转换车体角度 phase = theta - tracked_id × 2π/N，三种模型统一处理。

# 1.12
1. 发现 fire_control 反向依赖 hardware/serial 层，原因是 AimMode 枚举定义在 serial 层。依赖方向应该是 hardware → aimer，不能反过来。重构后 serial 层只保留 uint8_t aim_mode 原始字节，业务枚举 AimMode 定义在 aimer/common/，转换边界在 RobotState::from_sync_frame()。这样 fire_control 完全不知道 serial 层的存在。
2. 串口超时导致 FPS 异常低的问题。之前 recv_len == 0（超时无数据）会触发断线重连，实际上这是正常情况，只有 recv_len < 0（读取错误）才需要重连。修复后 SerialReceiver FPS 从 71 恢复到 500+。另外 UART 配置 VMIN=1 会导致无数据时 read() 永久阻塞，改为 VMIN=0 后 0.1s 超时返回。
3. 看门狗之前只能检测线程是否存活，无法区分"线程在跑但没有数据"。现在分离为 heartbeat() 和 heartbeat_data() 两种心跳，hardware 节点在串口有数据时才发 data 心跳。状态显示也改为 [OK 2ms] 形式，直观显示距上次心跳的时间。
4. fire_control 模块独立。通用逻辑（弹道解算、目标接口）提取到 aimer/fire_control/，auto_aim 专用逻辑（反陀螺、目标选择）留在 auto_aim/fire_control/。autoaim 用 VehicleTargetAdapter 适配，后续 autobuff 用 BladeTargetAdapter 适配。这样弹道解算代码完全不知道具体目标类型。
5. 命名空间加了 aimer:: 前缀（filter、math、tf、ballistic），避免与外部库冲突。之前 filter:: 和 Eigen 的 filter 模块冲突过。
6. 标定工具新增 ±90° 大范围搜索，125 个起点粗搜后迭代细化。适用于相机安装角度偏差较大的情况。
7. hardware 层应该只关心字节流，业务含义由上层定义。类似地，fire_control 应该只关心"目标在哪、速度多少"，不关心目标是装甲板还是扇叶。依赖方向错了会导致模块无法复用，改起来也很痛苦。
# 1.11
1. 完成 fire_control 模块独立重构，目的是让弹道解算、类型定义等通用代码可被 autoaim 和未来的 autobuff 共用。原先火控与 predictor 类型（VehicleState、ArmorState）紧耦合，导致能量机关无法复用弹道解算代码。
2. 采用适配器模式解耦：新建 TargetInterface 抽象接口，定义目标的通用属性（position、velocity、is_rotating 等）。autoaim 通过 VehicleTargetAdapter 将 VehicleState 适配为 TargetInterface。未来 autobuff 只需实现 BladeTargetAdapter 即可复用全部火控逻辑。
3. 目录结构调整：
3.1. 新建 aimer/fire_control/ 通用模块，包含 interface/（抽象接口）、core/types.hpp（通用类型）、core/trajectory/（弹道解算）
3.2. aimer/auto_aim/fire_control/ 保留 autoaim 专用逻辑（MPC、PID反陀螺、目标选择），其 types.hpp 改为重新导出通用类型 + autoaim 专用的 TargetSelection
3.3. trajectory/*.hpp 改为兼容层，重新导出 fire_control 版本
4. CMake 依赖链：fire_control_interface（header-only）→ fire_control_core（弹道解算，需 Ceres）→ autoaim_fire_control。旧的 fire_control target 保留为 alias 以兼容现有代码。
5. TargetInterface 扩展了多子目标支持：sub_target_count()、predict_sub_target_position(int idx, double dt)、predict_center(double dt)。这些方法原本是为 MPC 规划器设计的，但 MPC 模块暂未迁移，接口已预留。


# 1.10
1. 串口协议重构，支持配置文件选择协议。借鉴 librmcs 改进 UsbBulkProtocol，新增设备枚举、断线检测等功能。同时隔离依赖，协议工厂上层应用不应该依赖下层实现。在 hardware.toml 新增 Serial.protocol 配置项，可选 "uart" 或 "usb_bulk"，SerialManager 根据配置自动选择协议。同时修复了收发线程共享 _disconnected 标志的竞态问题，改用 std::atomic<bool>。
2. 看门狗脚本增强。添加资源监控功能，每 x 秒记录 RMCV 进程和系统的 CPU、内存、虚拟内存、温度到 resources.csv，用于赛后分析内存泄漏和性能瓶颈。添加崩溃时自动保存 core dump 功能，重启时搜索 build/、/tmp、/var/lib/systemd/coredump 等目录，将 core 文件移动到会话目录保存。
3. 录制节点修复。修复比赛模式（--match）未强制录制 raw/imu 的 bug，原因是 match_mode 检查逻辑放在了 writer 创建之后。优化 match_mode 查找逻辑，从循环内每帧查找改为循环外获取指针、循环内只读值。移除 debug video 录制功能，只保留 raw.mkv + imu.csv，简化录制流程。
4. 代码审查工具切换。移除 CodeRabbit 配置，改用 GitHub Copilot Code Review。

通过 TTL 发送出来的串口，除了稳态误差，随机误差很大。原因是数据在 Linux 的 tty 层会经过缓冲，不会立即发送/接收，而是等缓冲区满或超时（通常 1-10ms）才触发系统调用。加上 USB 转 TTL 芯片（CH340/CP2102 等）本身有 USB 轮询间隔（1ms），以及 Linux 非实时调度的抖动，导致收发时刻的随机误差可达数毫秒级别。
准备改用USB Bulk 协议绕过 tty 层，降低抖动

# 1.3
框架结构变成这样是不是更好：
  aimer/
  ├── common/                   # 基础库 (被所有模块依赖)
  │   ├── math/
  │   ├── filter/               # ← 滤波器放这里! 是通用工具
  │   │   └── adaptive_ekf.hpp
  │   └── transformer/
  │
  └── auto_aim/
      ├── detector/             # 检测模块
      │
      └── predictor/            # 预测模块
          ├── predictor_node.cpp    # 入口
          ├── enemy_predictor.cpp   # 协调器
          │
          ├── observer/             # 观测 (PnP)
          │   ├── armor_observer.cpp
          │   └── armor_table.hpp
          │
          └── model/                # 模型 (包含跟踪和滤波)
              ├── enemy_model.hpp       # 接口
              ├── vehicle/              # 车辆模型
              │   ├── vehicle_model.cpp
              │   ├── armor_tracker.cpp # ID分配+消抖
              │   └── motion/           # 运动EKF
              │       ├── armor_ekf.cpp
              │       ├── spin_ekf.cpp
              │       └── lmtd_ekf.cpp
              ├── outpost/
              └── base/

# 1.1元旦快乐

自瞄是辅助人类操作手的，因此我认为自瞄要打的应该是最靠相机中心的目标，因为操作手会把要击打的目标放在中心，因此你其实只要击打中心的目标就可以，操作手会按右键进行预瞄，也许我会给你发预瞄指令，此时电控也会进行抬枪，但是这也许又会导致最靠中心的目标改变，所以应该要有个锁

# 12.29

  异步并行推理时间线

  假设: 推理8ms, 相机5ms/帧, 队列4帧

  时间    相机帧     队列状态              GPU推理              输出
  ─────────────────────────────────────────────────────────────────────
  0ms     F0曝光    →push F0             [F0开始推理]
  5ms     F1曝光    →push F1             [F0推理中][F1排队]
  8ms                                    [F0完成]→输出F0      延迟=8ms
  10ms    F2曝光    →push F2             [F1开始][F2排队]
  15ms    F3曝光    →push F3             [F1推理中]
  16ms                                   [F1完成]→输出F1      延迟=11ms
  20ms    F4曝光    →push F4             [F2开始]
  24ms                                   [F2完成]→输出F2      延迟=14ms
  25ms    F5曝光    队列满,丢弃F5
  ...

  关键点

| 指标       | 值      | 说明               |
| ---------- | ------- | ------------------ |
| 第一帧延迟 | ~8ms    | 推理时间           |
| 稳态延迟   | ~8-32ms | 取决于队列深度     |
| 吞吐量     | ~125fps | 受限于GPU (1000/8) |
| 丢帧率     | ~37%    | (200-125)/200      |

  当前代码测量的延迟

  submit_time = push时刻（预处理后）
  latency_ms = pop时刻 - submit_time

  这个延迟包含: 排队时间 + 推理时间 + 后处理时间

  不包含: 相机曝光 → 图像传输 → 预处理（约2-3ms）

  真正的端到端延迟

  相机曝光 → 传输(~1ms) → 预处理(~1ms) → 排队(0-24ms) → 推理(8ms) → 后处理(~1ms)
                                                                      ↓
  总延迟: 约 11-35ms (取决于队列深度)

  结论: 并行不减少单帧延迟，只提高吞吐量。要降低延迟，需要减少队列深度或加快推理速度。

# 12.24

对于predictor和fire_controller,是一对多的关系，因此我不准备用发布订阅，我准备用BasicObjManager或者objmanager，那么我的tf是不是也可以用这个？这样子就不需要传两边参，像rmcvfans，或者有什么更好的解决方案？

对于/Users/david/Documents/GitHub/RMCV/config/aimer.toml里面的transformer，还有/Users/david/Documents/GitHub/RMCV/config/camera.yaml里的旋转矩阵，虽然我是让机械量的，但难免会有装配误差，这时候就需要一个一个尝试，显然不太现实，那能不能有个程序，能够把当前相机的xyz平移，比如+-1cm还有ypr旋转，比如+-10度都搜索一遍，然后选出最优的解，误差最小。比如就是按空格拍下相机前面静止的标定版，pnp测出距离，拍下多组，用opencv？eigen？ceres找出最优解？可以吗？

我可以假设我从枪管系发射了一连串子弹，然后他有自己的初速度和空气阻力，我想把它模拟出来然后画出来，这样子我可以知道我的枪管装的准不准，你可以20hz的发射频率，然后用空心圆把它画出来，并在左边打上发射出去的时候的时间戳

# 12.20

数据结构：
对于架构：用 aimer 作为顶层

```
  aimer/                              // 目录结构
  ├── common/                         // 公共类型
  ├── auto_aim/                       // 自瞄
  │   ├── detector/
  │   └── predictor/
  └── energy/                         // 能量机关

// 命名空间结构
namespace aimer {
// 公共类型 (RobotState, SyncFrame转换等)
struct RobotState { ... };
}
namespace autoaim {
  // 自瞄专用类型
  enum class ArmorType { ... };
  struct DetectedArmor { ... };

  namespace detector { ... }
  namespace predictor { ... }
}

namespace energy {
  // 能量机关
  namespace detector { ... }
  namespace predictor { ... }
}
```

# 12.18

数据流如下：
相机发布mat和时间辍，同时与接收线程的数据进行绑定，然后两个一起打包，而不是只打包imu。打包后发布
detector订阅相机发布，它需要相机发布里的图片和串口里的一些数据，比如颜色。进行检测，将检测后的vector,加上相机的内容发布出去（可以再记录一下时间辍）
TODO：predictor…

# 12.17

对于坐标转换器，我预想的是它能在predictor线程和fire_control线程中使用，这样的话它应该是以什么样的形式？如果以类的形式的话，在哪初始化比较好？在transform中，我希望它能存储相机内参，还有一些静态的转换比如陀螺仪的转换，但是如果我不创建类的话，我放在哪里比较好，对于这个转换器，我又不想对他加锁。对于动态的转换，在预测器线程，火控线程的订阅里，会有经过时间戳标定的四元数，它比实时获得的更精准，它与cvmat一一配对，因此陀螺仪四元数应该要从订阅内容的结构体中读，不采用hardware线程写入

# 12.16

基于12.9的架构，在现有自瞄基础上添加一个大地坐标系，此时再从串口自身接收车辆的速度信息，（加速度需要吗？）如果双方车辆运动，可以获得此时含自身速度的枪管坐标系和对方含速度的装甲板坐标，将二者放入大地坐标系下，对于子弹，不仅获得射速v，还获得车辆（枪管）的速度vx，vy，将其带入ceres，可求出射击的云台角度
TODO:把detector的相同头文件提出来

# 12.12

目前的想法是hardwarere节点->启动串口发送接收，同时启动相机，并将串口和相机做时间戳匹配
后续可以把toml参数读取逻辑从海康相机构造函数中剥离，放到hardware节点中，海康相机类中不需要知道什么是toml。它应该依赖于更抽象的vector或者struct

# 12.9

是不是可以解耦预测和选版逻辑，预测可以是100hz或者200hz，选版可以到500hz或者1000hz？后续验证一下可行性？

# 11.24

相机-陀螺仪时间戳标定？
一个yaw-pitch的二维云台
两个线程：相机是200hz采集，陀螺仪(是200hz采集)?
相机识别完物体后通过pnp产生了一个三维点pw，同时陀螺仪生成了一个四元数
其中物体是不动的，仅晃动云台，因此云台坐标系下的物体坐标应该是不动的
但其实相机时间戳和陀螺仪时间戳不一定完全对应，因此云台坐标系下的点可能会发生改变
所以我想找出一个deltat，使得云台不管怎么晃动云台坐标系下的点都是不动的
我有一个vector `<CamPoint>`和vector `<ImuData>`
struct ImuData {
    std::chrono::steady_clock::time_point timestamp;            // Chrono 稳定时间戳
    Eigen::Quaterniond orientation; // 姿态四元数 q = [w, x, y, z] (double精度)
};

struct CamPoint {
    std::chrono::steady_clock::time_point timestamp;    // Chrono 稳定时间戳
    cv::Point3d point_c;    // 相机坐标系下的三维点 (x, y, z) (double精度)
};
其中，campoint可以是识别装甲板得到的
或者说用chrono不好？全部改成用double？

可以用opencv，eigen，ceres
请设计一个巧妙的算法，来算出相机和陀螺仪之间的deltat，可以进行一些操作符重载和一些语法糖等，使用c++17
