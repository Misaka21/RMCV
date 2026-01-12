/**
 * @file fire_controller_node.hpp
 * @brief 统一火控节点入口
 *
 * 根据 AimMode 切换不同的控制器:
 *   - AUTOAIM: 自瞄控制器 (装甲板)
 *   - ENERGY_SMALL/LARGE: 能量机关控制器 (待实现)
 *   - DISABLED: 不控制
 */

#ifndef __AIMER_FIRE_CONTROL_FIRE_CONTROLLER_NODE_HPP__
#define __AIMER_FIRE_CONTROL_FIRE_CONTROLLER_NODE_HPP__

#include <string>

namespace fire_control {

/**
 * @brief 启动火控线程 (后台运行)
 *
 * @param config_path 配置文件路径 (如 "aimer.toml")
 */
void start_fire_control(const std::string& config_path);

/**
 * @brief 火控主循环 (阻塞)
 *
 * @param config_path 配置文件路径
 */
void fire_control_run(const std::string& config_path);

}  // namespace fire_control

#endif  // __AIMER_FIRE_CONTROL_FIRE_CONTROLLER_NODE_HPP__
