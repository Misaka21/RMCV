/**
 * @file fire_controller_node.hpp
 * @brief 火控线程节点
 *
 * 从 BasicObjManager<BattlefieldSnapshot> 获取数据
 * 输出 FireCommand 到 BasicObjManager
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_NODE_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_NODE_HPP__

#include <string>

namespace autoaim::fire_control {

/**
 * @brief 启动火控线程 (后台运行)
 *
 * @param config_path 配置文件路径
 */
void start_fire_control(const std::string& config_path);

/**
 * @brief 火控主循环 (阻塞)
 *
 * @param config_path 配置文件路径
 */
void fire_control_run(const std::string& config_path);

/**
 * @brief 后台运行火控 (100Hz 循环)
 *
 * @param config_path 配置文件路径
 */
void background_fire_control_run(const std::string& config_path);

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_NODE_HPP__
