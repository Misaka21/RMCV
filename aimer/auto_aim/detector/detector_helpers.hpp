//
// Detector Helpers - 检测节点辅助函数
// 提取 sync/async 模式的公共逻辑
//

#ifndef AIMER_AUTOAIM_DETECTOR_HELPERS_HPP
#define AIMER_AUTOAIM_DETECTOR_HELPERS_HPP

#include <opencv2/core/mat.hpp>

#include "aimer/common/types.hpp"
#include "common/detector_interface.hpp"
#include "common/types.hpp"
#include "hardware/serial/serial_thread.hpp"

namespace autoaim::detector {

// ============================================================================
// 颜色转换
// ============================================================================

/**
 * @brief 串口颜色值转换为检测器颜色枚举
 * @param serial_color 串口颜色 (0=未知, 1=红, 2=蓝)
 */
inline EnemyColor serial_to_enemy_color(uint8_t serial_color) {
    return (serial_color == 1) ? EnemyColor::RED : EnemyColor::BLUE;
}

// ============================================================================
// 检测结果构建
// ============================================================================

/**
 * @brief 从串口数据构建 RobotState
 */
inline aimer::RobotState build_robot_state(
    const serial::SerialReceiveData& s,
    int64_t timestamp_us
) {
    aimer::RobotState state;
    state.set_euler(s.yaw, s.pitch, s.roll);
    state.bullet_speed = s.bullet_speed;
    state.enemy_color = s.enemy_color;
    state.aim_mode = s.aim_mode;
    state.allow_fire = s.allow_fire;
    state.timestamp_us = timestamp_us;
    return state;
}

/**
 * @brief 构建检测结果 (同步模式用)
 */
inline aimer::DetectionResult build_detection_result(
    std::vector<DetectedArmor>&& armors,
    const cv::Mat& image,
    int frame_id,
    int64_t timestamp_us,
    const serial::SerialReceiveData& serial_data,
    float latency_ms
) {
    aimer::DetectionResult result;
    result.frame_id = frame_id;
    result.armors = std::move(armors);
    result.latency_ms = latency_ms;
    result.img = image;
    result.state = build_robot_state(serial_data, timestamp_us);
    return result;
}

/**
 * @brief 从异步结果构建检测结果
 */
inline aimer::DetectionResult build_detection_result(
    const AsyncDetectionResult& async_result
) {
    aimer::DetectionResult result;
    result.frame_id = async_result.frame_id;
    result.armors = async_result.armors;
    result.latency_ms = async_result.latency_ms;
    result.img = async_result.image;
    result.state = build_robot_state(async_result.serial_data, async_result.timestamp_us);
    return result;
}

// ============================================================================
// Debug 图像生成
// ============================================================================

/**
 * @brief 在图像上绘制检测结果 (Web调试用)
 */
cv::Mat draw_debug_overlay(
    const cv::Mat& image,
    const std::vector<DetectedArmor>& armors,
    float fps,
    float latency_ms
);

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_HELPERS_HPP
