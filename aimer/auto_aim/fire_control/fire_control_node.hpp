/**
 * @file fire_control_node.hpp
 * @brief 自瞄火控节点入口
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_NODE_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_NODE_HPP__

#include <string>

namespace autoaim::fire_control {

/**
 * @brief 启动火控线程 (后台运行)
 */
void start_fire_control_node(const std::string& config_path = "aimer.toml");

/**
 * @brief 火控主循环 (阻塞)
 */
void fire_control_run(const std::string& config_path);

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_NODE_HPP__
