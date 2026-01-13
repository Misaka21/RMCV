/**
 * @file types.hpp
 * @brief AutoAim 火控模块类型定义
 *
 * 此文件重新导出 fire_control 通用类型，并添加 autoaim 专用类型
 *
 * 通用类型来自: aimer/common/fire_control_types.hpp
 *   - GimbalState, LatencyInfo, AimResult, GimbalPlan, FireCommand
 *
 * AutoAim 专用类型:
 *   - TargetSelection (依赖 predictor)
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__

// 导入通用类型
#include "aimer/common/fire_control_types.hpp"

// AutoAim 专用依赖
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/robot_state.hpp"

namespace autoaim::fire_control {

// 从 fire_control 命名空间导入通用类型
using ::fire_control::GimbalState;
using ::fire_control::LatencyInfo;
using ::fire_control::AimResult;
using ::fire_control::GimbalPlan;
using ::fire_control::FireCommand;

// ==================== 常量定义 ====================

constexpr double CONTROL_DT = 0.01;        // 控制周期 10ms (100Hz)

// ==================== AutoAim 专用类型 ====================

/**
 * @brief 目标选择结果
 *
 * 包含指向 predictor 类型的指针，是 autoaim 专用类型
 */
struct TargetSelection {
    bool has_target = false;

    int target_id = -1;
    int armor_idx = -1;

    const predictor::VehicleState* vehicle = nullptr;
    const predictor::ArmorState* armor = nullptr;

    double priority = 0;       // 优先级评分

    // 插值后的位置 (考虑延迟)
    Eigen::Vector3d predicted_pos = Eigen::Vector3d::Zero();
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__
