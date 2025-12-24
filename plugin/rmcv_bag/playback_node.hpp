/**
 * @file playback_node.hpp
 * @brief 回放节点 - 从录制的 bag 读取数据
 */

#ifndef RMCV_BAG_PLAYBACK_NODE_HPP
#define RMCV_BAG_PLAYBACK_NODE_HPP

#include <string>

namespace rmcv_bag {

/**
 * @brief 启动回放节点
 *
 * 从指定目录读取 raw.mkv 和 imu.csv，模拟硬件输出
 * 发布消息: Message<hardware::SyncFrame> "sync_frame"
 *
 * @param bag_path 录制目录路径 (包含 raw.mkv 和 imu.csv)
 * @param playback_speed 回放速度倍率 (1.0 = 实时, 0 = 无延迟)
 */
void start_playback_node(const std::string& bag_path, double playback_speed = 1.0);

}  // namespace rmcv_bag

#endif  // RMCV_BAG_PLAYBACK_NODE_HPP
