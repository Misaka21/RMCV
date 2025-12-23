/**
 * @file armor_observer.cpp
 * @brief 装甲板观测器实现
 */

#include "armor_observer.hpp"

#include <cmath>

#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"

namespace autoaim::predictor {

const ArmorObservationTable& ArmorObserver::observe(
    const aimer::DetectionResult& detection,
    double timestamp
) {
    table_.clear();
    table_.set_frame(timestamp, ++frame_id_);

    const auto& q_imu = detection.state.q_imu;

    for (const auto& armor : detection.armors) {
        auto obs = solve_pnp(armor, timestamp, q_imu);
        if (obs.valid) {
            table_.add(obs);
        }
    }

    return table_;
}

ArmorObservation ArmorObserver::solve_pnp(
    const autoaim::DetectedArmor& armor,
    double timestamp,
    const Eigen::Quaterniond& q_imu
) {
    ArmorObservation obs;
    obs.valid = false;

    // 检查四角点数量
    if (armor.landmarks.size() != 4) {
        return obs;
    }

    // 使用 DetectedArmor 的模板点 (统一定义)
    std::vector<cv::Point3f> object_points = armor.object_points();

    // 2D 图像点
    std::vector<cv::Point2f> image_points(armor.landmarks.begin(), armor.landmarks.end());

    // 从 tf 模块获取相机内参
    const cv::Mat& camera_matrix = tf::get_camera_matrix();
    const cv::Mat& dist_coeffs = tf::get_distort_coeffs();

    // PnP 解算
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE
    );

    if (!success) {
        return obs;
    }

    // 提取位置 (相机坐标系)
    Eigen::Vector3d pos_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    // 计算装甲板法向量 (相机坐标系)
    // 物体坐标系: x前(指向相机), y左, z上
    // 法向量在物体坐标系是 [1, 0, 0] (x轴方向)
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    Eigen::Vector3d normal_cam(
        rotation_matrix.at<double>(0, 0),
        rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(2, 0)
    );

    // 计算 z_to_v (在相机系计算)
    double z_to_v = compute_z_to_v(pos_cam, normal_cam);

    // ========== 坐标变换: 相机系 → 世界系 ==========
    Eigen::Vector3d pos_world = tf::cam_to_world(pos_cam, q_imu);
    Eigen::Vector3d normal_world = tf::vector<tf::Frame::Camera, tf::Frame::World>(normal_cam, q_imu);

    // 计算观测向量 (世界系)
    Eigen::Vector4d z = compute_observation(pos_world, normal_world);

    // 构建观测结果 (世界系)
    obs = ArmorObservation::from_detection(armor, pos_world, z, z_to_v, timestamp);

    return obs;
}

Eigen::Vector4d ArmorObserver::compute_observation(
    const Eigen::Vector3d& pos_world,
    const Eigen::Vector3d& normal_world
) {
    Eigen::Vector4d z;

    // 计算球坐标 (世界系)
    double dist = pos_world.norm();
    double yaw = std::atan2(pos_world.y(), pos_world.x());
    double pitch = std::atan2(pos_world.z(), std::sqrt(pos_world.x() * pos_world.x() + pos_world.y() * pos_world.y()));

    // 装甲板朝向角 (法向量在 xy 平面的投影角度)
    double armor_yaw = std::atan2(normal_world.y(), normal_world.x());

    z << yaw, pitch, dist, armor_yaw;

    return z;
}

double ArmorObserver::compute_z_to_v(
    const Eigen::Vector3d& pos_cam,
    const Eigen::Vector3d& normal_cam
) {
    // 视线方向 (从相机指向装甲板)
    Eigen::Vector3d view_dir = pos_cam.normalized();

    // 装甲板法向量应该大致指向相机
    double cos_angle = -normal_cam.dot(view_dir);

    // 返回夹角 (弧度)
    return std::acos(std::clamp(cos_angle, -1.0, 1.0));
}

}  // namespace autoaim::predictor
