/**
 * @file recorder_node.hpp
 * @brief 录制节点 - 录制视频和传感器数据
 *
 * 订阅:
 *   - Message<hardware::SyncFrame> "sync_frame" (原始帧 + 串口数据)
 *
 * 输出文件 (保存到会话目录):
 *   - raw.mkv: 原始相机帧
 *   - imu.csv: 串口 IMU 数据
 */

#ifndef RMCV_BAG_RECORDER_NODE_HPP
#define RMCV_BAG_RECORDER_NODE_HPP

namespace rmcv_bag {

/**
 * @brief 启动录制节点
 *
 * 录制节点独立运行，通过运行时参数控制录制开关。
 * 配置项位于 config/recorder.toml 的 [Recorder] 节。
 */
void start_recorder_node();

}  // namespace rmcv_bag

#endif  // RMCV_BAG_RECORDER_NODE_HPP
