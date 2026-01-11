/**
 * @file rk4_trajectory_solver.hpp
 * @brief AutoAim RK4 弹道解算器 - 重新导出自 fire_control
 *
 * 此文件保留以兼容现有代码，实际实现在 fire_control/core/trajectory/
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_RK4_TRAJECTORY_SOLVER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_RK4_TRAJECTORY_SOLVER_HPP__

// 重新导出 fire_control RK4 解算器
#include "aimer/fire_control/core/trajectory/rk4_trajectory_solver.hpp"

namespace autoaim::fire_control {

// 导入 fire_control 类型
using ::fire_control::DragModel;
using ::fire_control::DragParams;
using ::fire_control::TrajectoryState;
using ::fire_control::TrajectoryResult;
using ::fire_control::Rk4TrajectorySolver;
using ::fire_control::SolverType;
using ::fire_control::solver_type_name;

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_RK4_TRAJECTORY_SOLVER_HPP__
