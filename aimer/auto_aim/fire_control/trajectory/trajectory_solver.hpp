/**
 * @file trajectory_solver.hpp
 * @brief AutoAim 弹道解算器 - 重新导出自 fire_control
 *
 * 此文件保留以兼容现有代码，实际实现在 fire_control/core/trajectory/
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_TRAJECTORY_SOLVER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_TRAJECTORY_SOLVER_HPP__

// 重新导出 fire_control 弹道解算器
#include "aimer/fire_control/core/trajectory/trajectory_solver.hpp"

namespace autoaim::fire_control {

// 导入 fire_control 类型
using ::fire_control::TrajectoryInput;
using ::fire_control::TrajectorySolverInterface;
using ::fire_control::LinearResistanceTrajectorySolver;
using ::fire_control::horizontal_distance;
using ::fire_control::distance_3d;

// 兼容别名
using TrajectorySolver = ::fire_control::LinearResistanceTrajectorySolver;

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_TRAJECTORY_SOLVER_HPP__
