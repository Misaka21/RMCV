# 12.12
目前的想法是hardwarere节点->启动串口发送接收，同时启动相机，并将串口和相机做时间戳匹配
后续可以把toml参数读取逻辑从海康相机构造函数中剥离，放到hardware节点中，海康相机类中不需要知道什么是toml。它应该依赖于更抽象的vector或者struct

# 12.9
是不是可以解耦预测和选版逻辑，预测可以是100hz或者200hz，选版可以到500hz或者1000hz？后续验证一下可行性？

# 11.24
相机-陀螺仪时间戳标定？
一个yaw-pitch的二维云台
两个线程：相机是100hz采集，陀螺仪是200hz采集
相机识别完物体后通过pnp产生了一个三维点pw，同时陀螺仪生成了一个四元数
其中物体是不动的，仅晃动云台，因此云台坐标系下的物体坐标应该是不动的
但其实相机时间戳和陀螺仪时间戳不一定完全对应，因此云台坐标系下的点可能会发生改变
所以我想找出一个deltat，使得云台不管怎么晃动云台坐标系下的点都是不动的
我有一个vector<CamPoint>和vector<ImuData>
struct ImuData {
    std::chrono::steady_clock::time_point timestamp;            // Chrono 稳定时间戳
    Eigen::Quaterniond orientation; // 姿态四元数 q = [w, x, y, z] (double精度)
};

struct CamPoint {
    std::chrono::steady_clock::time_point timestamp;    // Chrono 稳定时间戳
    cv::Point3d point_c;    // 相机坐标系下的三维点 (x, y, z) (double精度)
};
或者说用chrono不好？全部改成用double？




可以用opencv，eigen，ceres
请设计一个巧妙的算法，来算出相机和陀螺仪之间的deltat，可以进行一些操作符重载和一些语法糖等，使用c++17
