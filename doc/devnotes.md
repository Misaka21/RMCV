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
