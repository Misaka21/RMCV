/**
 * @file armor_observer.hpp
 * @brief 装甲板观测器
 *
 * 职责:
 * - PnP 解算 (相机坐标系)
 * - 坐标变换 (相机系 → 世界系)
 * - 观测向量计算
 * - 输出 ArmorObservationTable
 *
 * 数据流:
 *   DetectionResult + q_imu → ArmorObserver → ArmorObservationTable (世界系)
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_OBSERVER_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_OBSERVER_HPP__

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>

#include "aimer/auto_aim/common/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "armor_table.hpp"

namespace autoaim::predictor {

/**
 * @brief 装甲板观测器
 *
 * 负责将检测结果转换为世界坐标系的 3D 观测
 */
class ArmorObserver {
public:
    ArmorObserver();

    /**
     * @brief 处理检测结果，输出观测表 (世界坐标系)
     * @param detection 检测结果
     * @param q_imu IMU 四元数 (用于坐标变换)
     * @return 观测表
     */
    const ArmorObservationTable& observe(
        const autoaim::DetectionResult& detection,
        const Eigen::Quaterniond& q_imu
    );

    /**
     * @brief 加载相机内参
     */
    void set_camera_params(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);

    // 访问器
    const ArmorObservationTable& table() const { return table_; }
    double timestamp() const { return table_.timestamp(); }

private:
    /**
     * @brief 对单个装甲板做 PnP 解算并转换到世界系
     * @param armor 检测到的装甲板
     * @param timestamp 时间戳
     * @param q_imu IMU 四元数
     */
    ArmorObservation solve_pnp(
        const DetectedArmor& armor,
        double timestamp,
        const Eigen::Quaterniond& q_imu
    );

    /**
     * @brief 计算观测向量 [yaw, pitch, dist, armor_yaw] (世界系)
     */
    Eigen::Vector4d compute_observation(
        const Eigen::Vector3d& pos_world,
        const Eigen::Vector3d& normal_world
    );

    /**
     * @brief 计算 z_to_v (装甲板朝向与视线夹角，相机系)
     */
    double compute_z_to_v(
        const Eigen::Vector3d& pos_cam,
        const Eigen::Vector3d& normal_cam
    );

    // ==================== 数据 ====================

    // 观测表
    ArmorObservationTable table_;

    // 相机内参
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // 帧计数
    int frame_id_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_OBSERVER_HPP__
