//
// Dashboard Data - Dashboard数据结构定义
// 用于 UMT ObjManager 共享到 Python
//

#ifndef DASHBOARD_DATA_HPP
#define DASHBOARD_DATA_HPP

#include <string>

// Dashboard 数据结构
struct DashboardData {
    // Detector 数据
    float detect_latency_ms = 0;
    int armor_count = 0;
    std::string enemy_color = "UNKNOWN";
    std::string target_number = "";
    float target_x = 0;
    float target_y = 0;

    // IMU 数据
    float imu_yaw = 0;
    float imu_pitch = 0;
    float imu_roll = 0;

    // Serial 数据
    float bullet_speed = 0;
    int aim_mode = 0;
    bool allow_fire = false;
    bool serial_valid = false;

    // FPS 统计
    int detector_fps = 0;
    int hardware_fps = 0;
};

#endif  // DASHBOARD_DATA_HPP
