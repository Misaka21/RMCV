/**
 * @file armor_aim.hpp
 * @brief 装甲板瞄准逻辑 (统一处理陀螺/非陀螺)
 *
 * 当前策略（对齐 rm.cv.fans lmtd-top-model）:
 * - 非陀螺: direct-center（可见板）
 * - 陀螺:
 *   top0: direct(窗口内)；无 direct 则 idle
 *   top1/top2: direct(窗口内) -> indirect(等待板进入窗口)
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_PID_ARMOR_AIM_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_PID_ARMOR_AIM_HPP__

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "aimer/common/fire_control_types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

/**
 * @brief 瞄准模式
 */
enum class AimMode {
    DIRECT,     // direct-center
    INDIRECT    // emerging 等待板路径
};

/**
 * @brief 装甲板瞄准结果
 */
struct ArmorAimResult {
    bool valid = false;

    AimMode mode = AimMode::DIRECT;
    int armor_idx = -1;                 // 目标装甲板索引

    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();  // 瞄准位置
    Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();  // 目标速度 (用于速度前馈)

    double z_to_v = 0;                  // 装甲板朝向角 (用于开火判断)
    double time_to_fire = 0;            // INDIRECT 路径用于记录等待时长

    // 装甲板信息 (用于 FireDecision，避免传递 ArmorState* 指针)
    double armor_width = 0;
    double armor_height = 0;
};

/**
 * @brief 装甲板瞄准器
 *
 * 根据目标状态选择执行路径:
 * - 非陀螺: direct-center（可见板）
 * - 陀螺: 按 top0/top1/top2 配置做 direct/indirect 选择
 */
class ArmorAim {
public:
    ArmorAim() = default;

    /**
     * @brief 计算装甲板瞄准
     *
     * @param vehicle 目标车辆状态
     * @param predict_dt 预测时间 (弹道飞行时间)
     * @return 瞄准结果
     */
    ArmorAimResult compute(
        const predictor::VehicleState& vehicle,
        double predict_dt
    ) const;

    /**
     * @brief 计算装甲板瞄准（带云台状态）
     *
     * @param vehicle 目标车辆状态
     * @param predict_dt 预测时间
     * @param gimbal 当前云台状态（用于最小转动代价）
     * @param preferred_armor_idx 兼容入参（当前不作为直接决策因子）
     */
    ArmorAimResult compute(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        const Eigen::Quaterniond* q_imu,
        int preferred_armor_idx
    ) const;

    ArmorAimResult compute(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        int preferred_armor_idx
    ) const;

private:
    /**
     * @brief 非陀螺瞄准 (direct-center)
     */
    ArmorAimResult compute_non_spin(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        const Eigen::Quaterniond* q_imu,
        int preferred_armor_idx
    ) const;

    /**
     * @brief 陀螺瞄准 (rm.cv.fans: direct + indirect)
     */
    ArmorAimResult compute_spin(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        const Eigen::Quaterniond* q_imu,
        int preferred_armor_idx
    ) const;

    /**
     * @brief direct 执行路径
     *
     * @param visible_only true=仅可见板，false=可见+不可见预测板
     * @param strict_orientation_window true=窗口内无候选则直接失败（供 indirect 回退）
     */
    ArmorAimResult compute_direct(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        int preferred_armor_idx,
        bool use_orientation_window,
        double max_orientation_angle,
        bool visible_only,
        bool strict_orientation_window
    ) const;

    /**
     * @brief indirect 执行路径（rm.cv.fans emerging 思路）
     */
    ArmorAimResult compute_indirect(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        const Eigen::Quaterniond* q_imu,
        double max_orientation_angle,
        double max_out_error,
        int preferred_armor_idx
    ) const;

    /**
     * @brief 选择 DIRECT 模式的最佳装甲板
     *
     * 从可见装甲板中选择需要最小云台移动的那块
     */
    int choose_best_direct(
        const predictor::VehicleState& vehicle,
        const std::vector<int>& direct_indices,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        int preferred_armor_idx
    ) const;

    /**
     * @brief 计算装甲板速度 (用于速度前馈)
     *
     * v = ω × r (切向速度)
     */
    Eigen::Vector3d compute_armor_velocity(
        const predictor::VehicleState& vehicle,
        int armor_idx,
        double predict_dt
    ) const;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_PID_ARMOR_AIM_HPP__
