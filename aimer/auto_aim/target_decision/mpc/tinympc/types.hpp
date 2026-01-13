/**
 * @file types.hpp
 * @brief TinyMPC 类型定义
 *
 * 轻量级 MPC 求解器的核心数据结构
 * 改编自: https://github.com/TinyMPC/TinyMPC
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace tinympc {

using namespace Eigen;

using tinytype = double;
using tinyMatrix = Matrix<tinytype, Dynamic, Dynamic>;
using tinyVector = Matrix<tinytype, Dynamic, 1>;

/**
 * @brief MPC 求解结果
 */
struct TinySolution {
    int iter = 0;           // 迭代次数
    int solved = 0;         // 是否求解成功
    tinyMatrix x;           // 状态轨迹 (nx × N)
    tinyMatrix u;           // 控制轨迹 (nu × N-1)
};

/**
 * @brief 预计算缓存矩阵
 *
 * 包含 Riccati 递推的稳态解和预计算矩阵
 */
struct TinyCache {
    tinytype rho = 1.0;     // ADMM 惩罚系数
    tinyMatrix Kinf;        // 稳态反馈增益 (nu × nx)
    tinyMatrix Pinf;        // 稳态代价矩阵 (nx × nx)
    tinyMatrix Quu_inv;     // (R + B'PB)^-1 (nu × nu)
    tinyMatrix AmBKt;       // (A - BK)' (nx × nx)
    tinyVector APf;         // AmBKt * Pinf * f (nx × 1)
    tinyVector BPf;         // B' * Pinf * f (nu × 1)
};

/**
 * @brief 求解器设置
 */
struct TinySettings {
    tinytype abs_pri_tol = 1e-3;    // 原始残差容差
    tinytype abs_dua_tol = 1e-3;    // 对偶残差容差
    int max_iter = 1000;            // 最大迭代次数
    int check_termination = 1;      // 检查终止频率
    int en_state_bound = 1;         // 启用状态约束
    int en_input_bound = 1;         // 启用输入约束
};

/**
 * @brief 求解器工作空间
 */
struct TinyWorkspace {
    int nx = 0;             // 状态维度
    int nu = 0;             // 控制维度
    int N = 0;              // 预测时域

    // 状态和控制
    tinyMatrix x;           // 状态轨迹 (nx × N)
    tinyMatrix u;           // 控制轨迹 (nu × N-1)

    // 线性代价项
    tinyMatrix q;           // 状态代价 (nx × N)
    tinyMatrix r;           // 控制代价 (nu × N-1)

    // Riccati 后向传播项
    tinyMatrix p;           // (nx × N)
    tinyMatrix d;           // (nu × N-1)

    // ADMM 松弛变量
    tinyMatrix v;           // 状态松弛 (nx × N)
    tinyMatrix vnew;
    tinyMatrix z;           // 控制松弛 (nu × N-1)
    tinyMatrix znew;

    // ADMM 对偶变量
    tinyMatrix g;           // 状态对偶 (nx × N)
    tinyMatrix y;           // 控制对偶 (nu × N-1)

    // 约束边界
    tinyMatrix x_min;       // 状态下界 (nx × N)
    tinyMatrix x_max;       // 状态上界 (nx × N)
    tinyMatrix u_min;       // 控制下界 (nu × N-1)
    tinyMatrix u_max;       // 控制上界 (nu × N-1)

    // 系统矩阵 (用户提供)
    tinyVector Q;           // 状态代价对角线 (nx × 1)
    tinyVector R;           // 控制代价对角线 (nu × 1)
    tinyMatrix Adyn;        // 状态转移矩阵 (nx × nx)
    tinyMatrix Bdyn;        // 控制矩阵 (nx × nu)
    tinyVector fdyn;        // 仿射项 (nx × 1)

    // 参考轨迹
    tinyMatrix Xref;        // 状态参考 (nx × N)
    tinyMatrix Uref;        // 控制参考 (nu × N-1)

    // 临时变量
    tinyVector Qu;          // (nu × 1)

    // 求解状态
    tinytype primal_residual_state = 0;
    tinytype primal_residual_input = 0;
    tinytype dual_residual_state = 0;
    tinytype dual_residual_input = 0;
    int status = 0;
    int iter = 0;
};

/**
 * @brief TinyMPC 求解器主结构
 */
struct TinySolver {
    TinySolution* solution = nullptr;
    TinySettings* settings = nullptr;
    TinyCache* cache = nullptr;
    TinyWorkspace* work = nullptr;
};

}  // namespace tinympc
