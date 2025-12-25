/**
 * @file admm.hpp
 * @brief ADMM 求解器核心算法
 */

#pragma once

#include "types.hpp"

namespace tinympc {

/**
 * @brief ADMM 主求解函数
 * @return 0 成功, 1 失败
 */
int solve(TinySolver* solver);

/**
 * @brief Riccati 后向传播 (计算线性项)
 */
void backward_pass_grad(TinySolver* solver);

/**
 * @brief LQR 前向滚动 (计算轨迹)
 */
void forward_pass(TinySolver* solver);

/**
 * @brief 更新松弛变量 (投影到约束集)
 */
void update_slack(TinySolver* solver);

/**
 * @brief 更新对偶变量
 */
void update_dual(TinySolver* solver);

/**
 * @brief 更新线性代价项
 */
void update_linear_cost(TinySolver* solver);

/**
 * @brief 检查终止条件
 */
bool termination_condition(TinySolver* solver);

}  // namespace tinympc
