/**
 * @file visualizer_node.hpp
 * @brief 独立可视化线程，集中所有 OSD 绘制和 imshow
 *
 * 读取:
 *   BasicObjManager<BattlefieldSnapshot> "battlefield"
 *   BasicObjManager<FireCommand> "fire_command"
 * 显示:
 *   战场面板 + 延迟面板 + 火控面板 + EKF 调试图
 */

#ifndef __PLUGIN_VISUALIZER_VISUALIZER_NODE_HPP__
#define __PLUGIN_VISUALIZER_VISUALIZER_NODE_HPP__

namespace visualizer {

/**
 * @brief 启动可视化节点 (~30Hz)
 *
 * 热重载参数: Visualizer.show_window (bool)
 */
void start_visualizer_node();

}  // namespace visualizer

#endif  // __PLUGIN_VISUALIZER_VISUALIZER_NODE_HPP__
