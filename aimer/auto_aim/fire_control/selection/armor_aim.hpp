/**
 * @file armor_aim.hpp
 * @brief 装甲板瞄准逻辑 (统一处理陀螺/非陀螺)
 *
 * 当前策略:
 * - 仅在可见装甲板中选择执行板（始终 direct-center）
 * - 低速陀螺可启用 orientation 窗口硬筛选（无候选时回退到 direct-center）
 * - max_orientation_angle <= 0 或高速陀螺时，强制 direct-center
 * - 切板保持规则采用 keep_tracking_area_ratio（rm.cv.fans 风格）
 * - 不再使用 INDIRECT emerging 瞄准路径
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
    DIRECT,     // 直接跟踪当前可见装甲板
    INDIRECT    // 保留枚举值兼容旧调试字段（当前不会输出该模式）
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
    double time_to_fire = 0;            // 兼容字段，当前 direct-center 路径恒为 0

    // 装甲板信息 (用于 FireDecision，避免传递 ArmorState* 指针)
    double armor_width = 0;
    double armor_height = 0;
};

/**
 * @brief 装甲板瞄准器
 *
 * 根据目标状态选择可见执行板:
 * - 非陀螺: 可见板喵中心最小移动
 * - 低速陀螺: 先在 orientation 窗口内选可见板；若窗口内无可见板，回退可见板喵中心
 * - 高速陀螺: 直接可见板喵中心
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
     * @brief 计算装甲板瞄准（带云台状态，启用喵中心最小移动策略）
     *
     * @param vehicle 目标车辆状态
     * @param predict_dt 预测时间
     * @param gimbal 当前云台状态（用于最小转动代价）
     * @param preferred_armor_idx 上一帧装甲板索引（用于迟滞防抖）
     */
    ArmorAimResult compute(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        int preferred_armor_idx
    ) const;

private:
    /**
     * @brief 非陀螺瞄准 (直接跟踪推荐装甲板)
     */
    ArmorAimResult compute_non_spin(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        int preferred_armor_idx
    ) const;

    /**
     * @brief 陀螺瞄准 (direct-center + orientation 窗口软偏好)
     */
    ArmorAimResult compute_spin(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        int preferred_armor_idx
    ) const;

    /**
     * @brief 统一 direct-center 执行路径（仅可见板）
     */
    ArmorAimResult compute_direct_visible(
        const predictor::VehicleState& vehicle,
        double predict_dt,
        const ::fire_control::GimbalState* gimbal,
        int preferred_armor_idx,
        bool use_orientation_window,
        double max_orientation_angle
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
        int armor_idx
    ) const;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_PID_ARMOR_AIM_HPP__
