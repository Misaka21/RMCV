//
// 公共类型定义 - 数据流结构
// 定义各层级间传递的消息类型
//

#ifndef AIMER_COMMON_TYPES_HPP
#define AIMER_COMMON_TYPES_HPP

#include <chrono>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include "aimer/auto_aim/common/types.hpp"  // DetectedArmor, EnemyColor 等
#include "hardware/hardware_node.hpp"        // hardware::SyncFrame

namespace aimer {

using TimePoint = std::chrono::steady_clock::time_point;

// ============================================================================
// 1. 机器人状态 (RobotState)
// ============================================================================

/**
 * @brief 机器人状态 - 从 hardware::SyncFrame 提取
 *
 * 设计原则:
 * - 使用四元数表示姿态
 * - 时间戳为本机采集时间
 * - 透传到每个处理阶段
 */
struct RobotState {
    // IMU姿态 (云台坐标系 → 世界坐标系)
    Eigen::Quaterniond q_imu = Eigen::Quaterniond::Identity();

    // 自身速度 (世界坐标系, m/s) - 动打动用
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();

    // 弹速 (m/s)
    float bullet_speed = 15.0f;

    // 敌方颜色 (0=未知, 1=红, 2=蓝)
    uint8_t enemy_color = 0;

    // 自瞄模式 (0=关闭, 1=自瞄, 2=小符, 3=大符)
    uint8_t aim_mode = 0;

    // 是否允许射击
    bool allow_fire = false;

    // 本机采集时间戳
    TimePoint timestamp = {};

    RobotState() = default;

    // 从 hardware::SyncFrame 构建
    static RobotState from_sync_frame(const hardware::SyncFrame& frame) {
        RobotState state;
        if (frame.serial_valid) {
            const auto& s = frame.serial_data;
            state.set_euler(s.yaw, s.pitch, s.roll);
            state.bullet_speed = s.bullet_speed;
            state.enemy_color = s.enemy_color;
            state.aim_mode = s.aim_mode;
            state.allow_fire = s.allow_fire;
        }
        state.timestamp = frame.timestamp;
        return state;
    }

    // 从欧拉角构建四元数 (ZYX顺序, 输入为角度)
    void set_euler(float yaw_deg, float pitch_deg, float roll_deg) {
        constexpr double deg2rad = M_PI / 180.0;
        Eigen::AngleAxisd yaw_rot(yaw_deg * deg2rad, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitch_rot(pitch_deg * deg2rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd roll_rot(roll_deg * deg2rad, Eigen::Vector3d::UnitX());
        q_imu = yaw_rot * pitch_rot * roll_rot;
    }

    // 获取敌方颜色枚举
    autoaim::EnemyColor get_enemy_color() const {
        switch (enemy_color) {
            case 1: return autoaim::EnemyColor::RED;
            case 2: return autoaim::EnemyColor::BLUE;
            default: return autoaim::EnemyColor::WHITE;
        }
    }
};

// ============================================================================
// 2. 检测结果 (DetectionResult)
// ============================================================================

/**
 * @brief 检测结果 - Detector层输出
 *
 * 装甲板列表 + 透传的机器人状态
 */
struct DetectionResult {
    std::vector<autoaim::DetectedArmor> armors;
    int frame_id = 0;
    RobotState state;
    float latency_ms = 0;  // 检测耗时

    DetectionResult() = default;

    bool empty() const { return armors.empty(); }
    size_t size() const { return armors.size(); }
};

}  // namespace aimer

#endif  // AIMER_COMMON_TYPES_HPP
