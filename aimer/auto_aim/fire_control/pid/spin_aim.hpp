/**
 * @file spin_aim.hpp
 * @brief 反陀螺瞄准逻辑 (参考 rm.cv.fans top_model)
 *
 * 两种瞄准模式:
 * - DIRECT: 装甲板可见 (|z_to_v| < max_angle)，直接跟踪
 * - INDIRECT: 装甲板不可见，预判即将出现的位置 (emerging_pos)
 *
 * 适用于 PID 控制模式，不使用 MPC 的情况。
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_PID_SPIN_AIM_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_PID_SPIN_AIM_HPP__

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

/**
 * @brief 瞄准模式
 */
enum class AimMode {
    DIRECT,     // 直接跟踪当前可见装甲板
    INDIRECT    // 预判即将出现的装甲板位置
};

/**
 * @brief 反陀螺瞄准结果
 */
struct SpinAimResult {
    bool valid = false;

    AimMode mode = AimMode::DIRECT;
    int armor_idx = -1;                 // 目标装甲板索引

    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();  // 瞄准位置
    Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();  // 目标速度 (用于速度前馈)

    double z_to_v = 0;                  // 装甲板朝向角 (用于开火判断)
    double time_to_fire = 0;            // 到开火时机的时间 (s), INDIRECT 模式用
};

/**
 * @brief 反陀螺瞄准器
 *
 * 根据陀螺状态选择瞄准模式:
 * - 非陀螺: 直接跟踪推荐装甲板
 * - 陀螺 + 有可见板: DIRECT 模式
 * - 陀螺 + 无可见板: INDIRECT 模式 (预判)
 */
class SpinAim {
public:
    SpinAim() = default;

    /**
     * @brief 计算反陀螺瞄准
     *
     * @param vehicle 目标车辆状态
     * @param predict_dt 预测时间 (弹道飞行时间)
     * @return 瞄准结果
     */
    SpinAimResult compute(
        const predictor::VehicleState& vehicle,
        double predict_dt
    ) const;

private:
    /**
     * @brief 非陀螺瞄准 (直接跟踪推荐装甲板)
     */
    SpinAimResult compute_non_spin(
        const predictor::VehicleState& vehicle,
        double predict_dt
    ) const;

    /**
     * @brief 陀螺瞄准 (DIRECT 或 INDIRECT)
     */
    SpinAimResult compute_spin(
        const predictor::VehicleState& vehicle,
        double predict_dt
    ) const;

    /**
     * @brief 选择 DIRECT 模式的最佳装甲板
     *
     * 从可见装甲板中选择需要最小云台移动的那块
     */
    int choose_best_direct(
        const predictor::VehicleState& vehicle,
        const std::vector<int>& direct_indices
    ) const;

    /**
     * @brief 计算 INDIRECT 模式的瞄准位置
     *
     * 找到即将进入视野的装甲板，计算其"出现位置"
     */
    SpinAimResult compute_indirect(
        const predictor::VehicleState& vehicle,
        double predict_dt
    ) const;

    /**
     * @brief 计算装甲板速度 (用于速度前馈)
     *
     * v = ω × r (切向速度)
     */
    Eigen::Vector3d compute_armor_velocity(
        const predictor::VehicleState& vehicle,
        int armor_idx
    ) const;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_PID_SPIN_AIM_HPP__
