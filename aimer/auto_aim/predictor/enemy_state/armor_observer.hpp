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
 *   DetectionResult → ArmorObserver → ArmorObservationTable (世界系)
 *
 * 注意: 相机内参直接从 tf 模块获取 (tf::get_camera_matrix())
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_OBSERVER_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_OBSERVER_HPP__

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>

#include "aimer/common/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "armor_table.hpp"

namespace autoaim::predictor {

/**
 * @brief 装甲板观测器
 *
 * 负责将检测结果转换为世界坐标系的 3D 观测
 * 相机内参直接从 tf 模块获取，无需手动设置
 */
class ArmorObserver {
public:
    ArmorObserver() = default;

    /**
     * @brief 处理检测结果，输出观测表 (世界坐标系)
     * @param detection 检测结果
     * @param timestamp 当前时间戳 (s)
     * @return 观测表
     */
    const ArmorObservationTable& observe(
        const DetectionResult& detection,
        double timestamp
    );

    // 访问器
    const ArmorObservationTable& table() const { return table_; }
    double timestamp() const { return table_.timestamp(); }

private:
    /**
     * @brief 对单个装甲板做 PnP 解算并转换到世界系
     */
    ArmorObservation solve_pnp(
        const autoaim::DetectedArmor& armor,
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

    // 帧计数
    int frame_id_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_OBSERVER_HPP__
