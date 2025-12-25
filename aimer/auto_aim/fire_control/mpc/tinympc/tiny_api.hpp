/**
 * @file tiny_api.hpp
 * @brief TinyMPC 公共 API
 */

#pragma once

#include "admm.hpp"
#include "types.hpp"

namespace tinympc {

/**
 * @brief 初始化 TinyMPC 求解器
 *
 * @param solverp 求解器指针的指针
 * @param Adyn 状态转移矩阵 (nx × nx)
 * @param Bdyn 控制矩阵 (nx × nu)
 * @param fdyn 仿射项 (nx × 1)
 * @param Q 状态代价矩阵 (nx × nx, 对角)
 * @param R 控制代价矩阵 (nu × nu, 对角)
 * @param rho ADMM 惩罚系数
 * @param nx 状态维度
 * @param nu 控制维度
 * @param N 预测时域
 * @param verbose 是否打印调试信息
 * @return 0 成功
 */
int tiny_setup(
    TinySolver** solverp,
    const tinyMatrix& Adyn,
    const tinyMatrix& Bdyn,
    const tinyVector& fdyn,
    const tinyMatrix& Q,
    const tinyMatrix& R,
    tinytype rho,
    int nx,
    int nu,
    int N,
    int verbose = 0
);

/**
 * @brief 设置约束边界
 */
int tiny_set_bound_constraints(
    TinySolver* solver,
    const tinyMatrix& x_min,
    const tinyMatrix& x_max,
    const tinyMatrix& u_min,
    const tinyMatrix& u_max
);

/**
 * @brief 预计算缓存矩阵 (Riccati 稳态解)
 */
int tiny_precompute_cache(
    TinyCache* cache,
    const tinyMatrix& Adyn,
    const tinyMatrix& Bdyn,
    const tinyVector& fdyn,
    const tinyMatrix& Q,
    const tinyMatrix& R,
    int nx,
    int nu,
    tinytype rho,
    int verbose = 0
);

/**
 * @brief 求解 MPC 问题
 */
int tiny_solve(TinySolver* solver);

/**
 * @brief 设置初始状态
 */
int tiny_set_x0(TinySolver* solver, const tinyVector& x0);

/**
 * @brief 设置状态参考轨迹
 */
int tiny_set_x_ref(TinySolver* solver, const tinyMatrix& x_ref);

/**
 * @brief 设置控制参考轨迹
 */
int tiny_set_u_ref(TinySolver* solver, const tinyMatrix& u_ref);

/**
 * @brief 释放求解器内存
 */
void tiny_cleanup(TinySolver* solver);

}  // namespace tinympc
