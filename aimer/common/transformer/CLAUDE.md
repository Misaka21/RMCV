# transformer 说明

本目录负责坐标系转换。它是 detector、predictor、fire_control 之间共享空间语义的
基础，修改时要特别谨慎。

## 初始化

主程序通过以下方式初始化：

```cpp
aimer::tf::init("camera.yaml");
```

配置来自 `config/camera.yaml`。

## 坐标系

- `World`: 上电时云台位置，X 前、Y 左、Z 上。
- `Imu`: IMU 芯片坐标系。
- `Gimbal`: 云台坐标系，X 前、Y 左、Z 上。
- `Camera`: 相机坐标系，X 右、Y 下、Z 前。
- `Barrel`: 枪口坐标系，X 前、Y 左、Z 上。

## 改动规则

- 不要在业务模块里手写重复坐标变换，优先补充 transformer 接口。
- 修改坐标定义、外参方向、矩阵乘法顺序时，必须同步更新 README 和测试。
- 相机外参是静态参数，运行期热更新参数不要混进 transformer 初始化数据。

## 验证

```bash
./build/test_transformer
./build/test_gimbal2imubody
./build/test_ground_plane
```
