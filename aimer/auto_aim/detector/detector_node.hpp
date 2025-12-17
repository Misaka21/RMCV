//
// Detector Node - 检测节点
// 订阅sync_frame，运行装甲板检测，发布检测结果
//

#ifndef DETECTOR_NODE_HPP
#define DETECTOR_NODE_HPP

#include <chrono>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "traditional/types.hpp"

namespace autoaim {

using TimePoint = std::chrono::steady_clock::time_point;

/**
 * @brief Detection result for a single frame
 */
struct DetectionResult {
    int frame_id = 0;
    TimePoint timestamp;
    std::vector<detector::Armor> armors;
    float detect_latency_ms = 0;  // Detection time in ms
};

/**
 * @brief Start detector node
 *
 * This function:
 * 1. Loads detector from config/detector.toml
 * 2. Subscribes to "sync_frame" channel
 * 3. Runs detection on each frame
 * 4. Publishes DetectionResult to "detection_result" channel
 *
 * @param color Enemy color to detect (RED or BLUE)
 */
void start_detector_node(detector::EnemyColor color);

/**
 * @brief Set enemy color at runtime
 */
void set_enemy_color(detector::EnemyColor color);

}  // namespace autoaim

#endif  // DETECTOR_NODE_HPP
