/**
 * @file predictor_node.hpp
 * @brief 预测器节点接口
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_PREDICTOR_NODE_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_PREDICTOR_NODE_HPP__

namespace autoaim::predictor {

/**
 * @brief 启动预测器节点
 *
 * 订阅: Message<aimer::DetectionResult> "detections"
 * 发布: Message<BattlefieldSnapshot> "battlefield"
 */
void start_predictor_node();

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_PREDICTOR_NODE_HPP__
