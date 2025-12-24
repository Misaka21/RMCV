//
// 机器人状态定义
//

#ifndef AIMER_COMMON_ROBOT_STATE_HPP
#define AIMER_COMMON_ROBOT_STATE_HPP

#include <cmath>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hardware/hardware_node.hpp"

namespace aimer {

/**
 * @brief 机器人状态 - 从 hardware::SyncFrame 提取
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

    // 时间戳 (微秒, steady_clock)
    int64_t timestamp_us = 0;

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
        state.timestamp_us = frame.timestamp_us;
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
};

}  // namespace aimer

#endif  // AIMER_COMMON_ROBOT_STATE_HPP
