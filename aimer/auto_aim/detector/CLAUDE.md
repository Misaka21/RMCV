# auto_aim detector 说明

本目录负责装甲板检测和检测结果构建。

## 入口与输出

- 入口线程：`detector_node.cpp`
- 检测器工厂：`detector_factory.*`
- 统一接口：`common/detector_interface.hpp`
- 输出消息：`umt::Publisher<DetectionResult>("detections")`
- 调试图像：`"/detector/debug"` 和 `BasicObjManager<cv::Mat>("detector_debug_img")`

## 检测器实现

- `detector_rv/`: 传统视觉装甲板检测。
- `detector_ov/`: OpenVINO 检测器。
- `detector_trt/`: TensorRT 检测器和 CUDA 预处理。

## 业务规则

- 非 `AimMode::AUTOAIM` 时跳过自瞄检测。
- 敌方颜色来自串口输入，`0` 表示未知，未知时保持检测器当前颜色。
- detector 可以统计延迟、画 overlay、发送 Rerun 数据，但不要把调试输出变成主流程依赖。
- detection result 必须携带图像帧时间戳和 `RobotState`，供 predictor 和火控延迟补偿使用。

## 修改注意

- 同步检测器和异步检测器都要维持相同输出语义。
- 修改 armor 类型或角点定义时，同步检查 predictor observer、火控选择逻辑和调试绘制。
- TensorRT/OpenVINO 依赖可能不是所有机器都有，CMake 改动要保持可选构建。
