/**
 * @file trajectory_solver_factory.hpp
 * @brief AutoAim 弹道解算器工厂 - 重新导出自 fire_control
 *
 * 此文件保留以兼容现有代码，实际实现在 fire_control/core/trajectory/
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_TRAJECTORY_SOLVER_FACTORY_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_TRAJECTORY_SOLVER_FACTORY_HPP__

// 重新导出 fire_control 工厂
#include "aimer/fire_control/core/trajectory/solver_factory.hpp"

namespace autoaim::fire_control {

// 导入 trajectory 命名空间
namespace trajectory = ::fire_control::trajectory;

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_TRAJECTORY_SOLVER_FACTORY_HPP__
