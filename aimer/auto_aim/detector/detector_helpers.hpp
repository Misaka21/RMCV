//
// Detector Helpers - 检测节点辅助函数
// 包含数据转换和可视化功能
//

#ifndef AIMER_AUTOAIM_DETECTOR_HELPERS_HPP
#define AIMER_AUTOAIM_DETECTOR_HELPERS_HPP

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include "aimer/common/types.hpp"
#include "common/detector_interface.hpp"
#include "hardware/hardware_node.hpp"
#include "hardware/serial/serial_thread.hpp"

namespace autoaim::detector {

// ============================================================================
// 数据转换
// ============================================================================

/**
 * @brief 串口颜色值转换为检测器颜色枚举
 * @param serial_color 串口颜色 (0=未知, 1=红, 2=蓝)
 */
inline EnemyColor serial_to_enemy_color(uint8_t serial_color) {
    return (serial_color == 1) ? EnemyColor::RED : EnemyColor::BLUE;
}

/**
 * @brief 从串口数据构建 RobotState
 */
inline aimer::RobotState build_robot_state(
    const serial::SerialReceiveData& s,
    int64_t timestamp_us
) {
    aimer::RobotState state;
    // 新协议: yaw/pitch/roll 已经是弧度
    state.set_euler_rad(s.yaw, s.pitch, s.roll);
    state.bullet_speed = s.bullet_speed;
    state.aim_mode = aimer::to_aim_mode(s.aim_mode);  // uint8_t → AimMode
    state.aiming_lock = s.aiming_lock;
    state.enemy_color = s.enemy_color;
    state.allow_fire = true;
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
// 可视化 - 公共绘制
// ============================================================================

/**
 * @brief 在图像上绘制装甲板
 * @param img 要绘制的图像 (会被修改)
 * @param armors 装甲板列表
 * @param colorful 是否使用彩色 (true: 按敌方颜色, false: 统一绿色)
 */
void draw_armors(
    cv::Mat& img,
    const std::vector<DetectedArmor>& armors,
    bool colorful = false
);

// ============================================================================
// 可视化 - Web 调试 (轻量)
// ============================================================================

/**
 * @brief 生成 Web 调试图像
 *
 * 简单绘制: 装甲板轮廓 + FPS/延迟信息
 * 用于 Web 推流，不包含复杂的世界坐标网格
 */
cv::Mat draw_debug_overlay(
    const cv::Mat& image,
    const std::vector<DetectedArmor>& armors,
    float fps,
    float latency_ms
);

// ============================================================================
// 可视化 - 本地调试 (完整)
// ============================================================================

/**
 * @brief 绘制世界坐标系地面网格
 *
 * 用于验证坐标变换是否正确
 *
 * @param img 要绘制的图像
 * @param q_imu IMU四元数
 * @param grid_size 网格间距 (米)
 * @param range 绘制范围 (米)
 * @param ground_z 地面高度 (米，相对于云台)
 */
void draw_world_ground_grid(
    cv::Mat& img,
    const Eigen::Quaterniond& q_imu,
    double grid_size = 0.5,
    double range = 10.0,
    double ground_z = -0.5
);

/**
 * @brief 绘制完整调试可视化并显示窗口
 *
 * 包含: 世界坐标网格 + 装甲板 + IMU信息
 * 用于本地 cv::imshow 调试
 */
void draw_debug_visualization(
    const cv::Mat& image,
    const aimer::DetectionResult& result,
    const hardware::SyncFrame& frame
);

/**
 * @brief 关闭调试窗口
 */
void close_debug_window();

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_HELPERS_HPP
