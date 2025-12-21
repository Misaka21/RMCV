//
// Created by nuc11 on 2025/10/5.
//

#include "transformer.hpp"

#include <opencv2/core/persistence.hpp>

#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace tf {

static cv::Mat g_camera_matrix;
static cv::Mat g_distort_coeffs;
static Eigen::Vector3d g_robot_position = Eigen::Vector3d::Zero();
static Eigen::Vector3d g_last_v_world = Eigen::Vector3d::Zero();
static bool g_has_last_v = false;

// Camera → Gimbal 变换矩阵 (平移从TOML动态读取)
Eigen::Matrix4d Transform<Frame::Camera, Frame::Gimbal>::get(const Eigen::Quaterniond&) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R_;
    T.block<3, 1>(0, 3) = Eigen::Vector3d(
        runtime_param::get_param<double>("Transformer.camera_offset_x"),
        runtime_param::get_param<double>("Transformer.camera_offset_y"),
        runtime_param::get_param<double>("Transformer.camera_offset_z")
    );
    return T;
}

// Barrel → Gimbal 变换矩阵 (偏移从TOML动态读取)
Eigen::Matrix4d Transform<Frame::Barrel, Frame::Gimbal>::get(const Eigen::Quaterniond&) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) = Eigen::Vector3d(
        runtime_param::get_param<double>("Transformer.barrel_offset_x"),
        runtime_param::get_param<double>("Transformer.barrel_offset_y"),
        runtime_param::get_param<double>("Transformer.barrel_offset_z")
    );
    return T;
}

bool init(const std::string& yaml_file) {
    std::string yaml_path = std::string(CONFIG_DIR) + "/" + yaml_file;
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        debug::print(debug::PrintMode::ERROR, "Transformer", "Failed to open: {}", yaml_path);
        return false;
    }

    // 相机内参
    std::vector<double> cam_data;
    fs["camera_matrix"] >> cam_data;
    if (cam_data.size() == 9) {
        g_camera_matrix = cv::Mat(3, 3, CV_64F);
        std::memcpy(g_camera_matrix.data, cam_data.data(), 9 * sizeof(double));
    }

    // 畸变系数
    std::vector<double> dist_data;
    fs["distort_coeffs"] >> dist_data;
    if (!dist_data.empty()) {
        g_distort_coeffs = cv::Mat(1, static_cast<int>(dist_data.size()), CV_64F);
        std::memcpy(g_distort_coeffs.data, dist_data.data(), dist_data.size() * sizeof(double));
    }

    // R_gimbal2imubody: Gimbal → Imu (静态)
    std::vector<double> r_gimbal_data;
    fs["R_gimbal2imubody"] >> r_gimbal_data;
    if (r_gimbal_data.size() == 9) {
        Eigen::Matrix3d R;
        R << r_gimbal_data[0], r_gimbal_data[1], r_gimbal_data[2],
             r_gimbal_data[3], r_gimbal_data[4], r_gimbal_data[5],
             r_gimbal_data[6], r_gimbal_data[7], r_gimbal_data[8];
        Transform<Frame::Gimbal, Frame::Imu>::set_rotation(R);
    }

    // R_camera2gimbal: Camera → Gimbal (静态)
    std::vector<double> r_cam_data;
    fs["R_camera2gimbal"] >> r_cam_data;
    if (r_cam_data.size() == 9) {
        Eigen::Matrix3d R;
        R << r_cam_data[0], r_cam_data[1], r_cam_data[2],
             r_cam_data[3], r_cam_data[4], r_cam_data[5],
             r_cam_data[6], r_cam_data[7], r_cam_data[8];
        Transform<Frame::Camera, Frame::Gimbal>::set_rotation(R);
    }

    fs.release();
    debug::print(debug::PrintMode::INFO, "Transformer", "Loaded from {}", yaml_path);
    return true;
}

const cv::Mat& get_camera_matrix() { return g_camera_matrix; }
const cv::Mat& get_distort_coeffs() { return g_distort_coeffs; }

void update_odometry(const Eigen::Vector3d& v_gimbal, double dt, const Eigen::Quaterniond& q_imu) {
    Eigen::Vector3d v_world = vector<Frame::Gimbal, Frame::World>(v_gimbal, q_imu);

    if (g_has_last_v) {
        // 梯形积分: 用前后速度的平均值
        g_robot_position += (g_last_v_world + v_world) * 0.5 * dt;
    } else {
        // 第一次调用，用欧拉积分
        g_robot_position += v_world * dt;
        g_has_last_v = true;
    }
    g_last_v_world = v_world;
}

const Eigen::Vector3d& get_robot_position() { return g_robot_position; }

void reset_odometry() {
    g_robot_position.setZero();
    g_last_v_world.setZero();
    g_has_last_v = false;
}

}  // namespace tf
