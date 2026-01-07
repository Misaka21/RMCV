//
// Detector Node - 检测节点
// 订阅 sync_frame，运行装甲板检测，发布检测结果
//
// 根据检测器类型自动选择模式:
//   - is_async() = false: 单线程同步模式 (传统检测器)
//   - is_async() = true:  双线程异步模式 (YOLO检测器)
//

#ifndef AIMER_AUTOAIM_DETECTOR_NODE_HPP
#define AIMER_AUTOAIM_DETECTOR_NODE_HPP

namespace autoaim {

/**
 * @brief 启动检测节点
 *
 * 订阅: Message<hardware::SyncFrame> "sync_frame"
 * 发布: Message<aimer::DetectionResult> "detections"
 * 发布: Message<cv::Mat> "/detector/debug" (Web调试图像)
 */
void start_detector_node();

}  // namespace autoaim

#endif  // AIMER_AUTOAIM_DETECTOR_NODE_HPP
