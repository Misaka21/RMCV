//
// Dashboard Data - pybind11 导出
// 使用 UMT ObjManager 直接共享到 Python
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>

#include "dashboard_data.hpp"
#include "umt/umt.hpp"

namespace py = pybind11;

// 导出 DashboardData 到 Python
UMT_EXPORT_OBJMANAGER_ALIAS(DashboardData, DashboardData, c) {
    c.def(py::init<>());

    // Detector 数据
    c.def_readwrite("detect_latency_ms", &DashboardData::detect_latency_ms);
    c.def_readwrite("armor_count", &DashboardData::armor_count);
    c.def_readwrite("enemy_color", &DashboardData::enemy_color);
    c.def_readwrite("target_number", &DashboardData::target_number);
    c.def_readwrite("target_x", &DashboardData::target_x);
    c.def_readwrite("target_y", &DashboardData::target_y);

    // IMU 数据
    c.def_readwrite("imu_yaw", &DashboardData::imu_yaw);
    c.def_readwrite("imu_pitch", &DashboardData::imu_pitch);
    c.def_readwrite("imu_roll", &DashboardData::imu_roll);

    // Serial 数据
    c.def_readwrite("bullet_speed", &DashboardData::bullet_speed);
    c.def_readwrite("aim_mode", &DashboardData::aim_mode);
    c.def_readwrite("allow_fire", &DashboardData::allow_fire);
    c.def_readwrite("serial_valid", &DashboardData::serial_valid);

    // FPS 统计
    c.def_readwrite("detector_fps", &DashboardData::detector_fps);
    c.def_readwrite("hardware_fps", &DashboardData::hardware_fps);
}
