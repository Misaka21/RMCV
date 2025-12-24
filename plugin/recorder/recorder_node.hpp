/**
 * @file recorder_node.hpp
 * @brief 录制节点 - 录制视频和传感器数据
 *
 * 订阅:
 *   - Message<hardware::SyncFrame> "sync_frame" (原始帧 + 串口数据)
 *   - Message<cv::Mat> "predictor_vis" (predictor可视化输出)
 *
 * 输出文件 (保存到会话目录):
 *   - raw.avi: 原始相机帧
 *   - debug.avi: predictor 可视化帧
 *   - imu.csv: 串口 IMU 数据
 */

#ifndef PLUGIN_RECORDER_RECORDER_NODE_HPP
#define PLUGIN_RECORDER_RECORDER_NODE_HPP

#include <string>

namespace recorder {

/**
 * @brief 启动录制节点
 *
 * 录制节点独立运行，通过运行时参数控制录制开关。
 * 配置项位于 config/aimer.toml 的 [Recorder] 节。
 */
void start_recorder_node();

}  // namespace recorder

#endif  // PLUGIN_RECORDER_RECORDER_NODE_HPP
