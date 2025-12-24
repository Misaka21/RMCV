/**
 * @file armor_observer.hpp
 * @brief 装甲板观测器
 *
 * 职责:
 * - PnP 解算 (相机坐标系)
 * - 坐标变换 (相机系 → 世界系)
 * - 三分法优化 z_to_v (装甲板朝向角)
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

#include <array>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>

#include "aimer/common/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "armor_table.hpp"

namespace autoaim::predictor {

// 三分法迭代次数，12次约0.5度精度
constexpr int FIT_Z_TO_V_ITERATIONS = 15;

/**
 * @brief 装甲板观测器
 *
 * 负责将检测结果转换为世界坐标系的 3D 观测
 * 包含三分法优化 z_to_v，提高装甲板朝向角精度
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

    // ==================== 三分法优化 z_to_v ====================

    /**
     * @brief 对检测点进行畸变矫正
     * @param pts 原始四角点
     * @return 畸变矫正后的四角点
     */
    std::array<cv::Point2f, 4> undistort_points(
        const std::vector<cv::Point2f>& pts
    );

    /**
     * @brief 给定 z_to_v 计算装甲板四角点在图像上的投影
     * @param pos 装甲板中心 (相机系)
     * @param type 装甲板类型
     * @param pitch 装甲板俯仰角
     * @param z_to_v 装甲板法向与相机视线的夹角
     * @return 投影的四角点
     */
    std::array<cv::Point2f, 4> project_armor_corners(
        const Eigen::Vector3d& pos,
        ArmorType type,
        double pitch,
        double z_to_v
    );

    /**
     * @brief 计算重投影代价
     * @param projected 模型投影的四角点
     * @param detected 实际检测的四角点 (畸变矫正后)
     * @param z_to_v 当前的 z_to_v (用于权重调整)
     * @return 代价值
     */
    double compute_reprojection_cost(
        const std::array<cv::Point2f, 4>& projected,
        const std::array<cv::Point2f, 4>& detected,
        double z_to_v
    );

    /**
     * @brief 三分搜索找到最优的 z_to_v
     * @param pos 装甲板中心 (相机系)
     * @param type 装甲板类型
     * @param pitch 装甲板俯仰角
     * @param pus 畸变矫正后的检测四角点
     * @param z_to_v_init 初始估计 (PnP 结果)
     * @return 优化后的 z_to_v
     */
    double fit_z_to_v(
        const Eigen::Vector3d& pos,
        ArmorType type,
        double pitch,
        const std::array<cv::Point2f, 4>& pus,
        double z_to_v_init
    );

    // ==================== 数据 ====================

    // 观测表
    ArmorObservationTable table_;

    // 帧计数
    int frame_id_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_OBSERVER_HPP__
