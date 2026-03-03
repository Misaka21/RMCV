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

constexpr double CONTROL_DT = 0.002;        // 控制周期 2ms (500Hz)

// ==================== AutoAim 专用类型 ====================

/**
 * @brief 目标选择结果
 *
 * 只包含敌人索引，不包含装甲板索引。
 * 装甲板选择由 ArmorAim 负责。
 *
 * 职责边界:
 * - TargetSelector: 选"打哪个敌人" → target_id
 * - ArmorAim: 选"打这个敌人的哪块板" → armor_idx, target_pos
 */
struct TargetSelection {
    bool has_target = false;

    int target_id = -1;        // 敌人索引 (snapshot.vehicles[target_id])

    double priority = 0;       // 优先级评分

    // 用于调试显示的预测位置 (使用 recommended_armor_idx)
    Eigen::Vector3d predicted_pos = Eigen::Vector3d::Zero();
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_TYPES_HPP__
