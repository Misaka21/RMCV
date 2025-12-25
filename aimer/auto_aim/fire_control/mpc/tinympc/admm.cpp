/**
 * @file admm.cpp
 * @brief ADMM 求解器核心算法实现
 *
 * 改编自 TinyMPC 项目
 */

#include "admm.hpp"

#include <algorithm>
#include <cmath>

namespace tinympc {

void backward_pass_grad(TinySolver* solver)
{
    auto* work = solver->work;
    auto* cache = solver->cache;

    for (int i = work->N - 2; i >= 0; --i) {
        // d[i] = Quu_inv * (B' * p[i+1] + r[i] + B' * Pinf * f)
        work->d.col(i).noalias() = cache->Quu_inv * (
            work->Bdyn.transpose() * work->p.col(i + 1) +
            work->r.col(i) +
            cache->BPf
        );

        // p[i] = q[i] + (A - BK)' * p[i+1] - K' * r[i] + (A - BK)' * Pinf * f
        work->p.col(i).noalias() = work->q.col(i) +
            cache->AmBKt * work->p.col(i + 1) -
            cache->Kinf.transpose() * work->r.col(i) +
            cache->APf;
    }
}

void forward_pass(TinySolver* solver)
{
    auto* work = solver->work;
    auto* cache = solver->cache;

    for (int i = 0; i < work->N - 1; ++i) {
        // u[i] = -K * x[i] - d[i]
        work->u.col(i).noalias() = -cache->Kinf * work->x.col(i) - work->d.col(i);

        // x[i+1] = A * x[i] + B * u[i] + f
        work->x.col(i + 1).noalias() = work->Adyn * work->x.col(i) +
                                        work->Bdyn * work->u.col(i) +
                                        work->fdyn;
    }
}

void update_slack(TinySolver* solver)
{
    auto* work = solver->work;
    auto* settings = solver->settings;

    // 状态松弛变量
    work->vnew = work->x + work->g;

    // 控制松弛变量
    work->znew = work->u + work->y;

    // 状态约束投影 (box constraints)
    if (settings->en_state_bound) {
        work->vnew = work->x_max.cwiseMin(work->x_min.cwiseMax(work->vnew));
    }

    // 控制约束投影 (box constraints)
    if (settings->en_input_bound) {
        work->znew = work->u_max.cwiseMin(work->u_min.cwiseMax(work->znew));
    }
}

void update_dual(TinySolver* solver)
{
    auto* work = solver->work;

    // 状态对偶变量
    work->g = work->g + work->x - work->vnew;

    // 控制对偶变量
    work->y = work->y + work->u - work->znew;
}

void update_linear_cost(TinySolver* solver)
{
    auto* work = solver->work;
    auto* cache = solver->cache;

    // 状态代价项: q = -Q * Xref - rho * (vnew - g)
    work->q = -(work->Xref.array().colwise() * work->Q.array());
    work->q.noalias() -= cache->rho * (work->vnew - work->g);

    // 控制代价项: r = -R * Uref - rho * (znew - y)
    work->r = -(work->Uref.array().colwise() * work->R.array());
    work->r.noalias() -= cache->rho * (work->znew - work->y);

    // 终端代价
    work->p.col(work->N - 1) = -(work->Xref.col(work->N - 1).transpose() * cache->Pinf);
    work->p.col(work->N - 1).noalias() -= cache->rho * (
        work->vnew.col(work->N - 1) - work->g.col(work->N - 1)
    );
}

bool termination_condition(TinySolver* solver)
{
    auto* work = solver->work;
    auto* settings = solver->settings;
    auto* cache = solver->cache;

    if (work->iter % settings->check_termination == 0) {
        // 计算原始残差
        work->primal_residual_state = (work->x - work->vnew).cwiseAbs().maxCoeff();
        work->primal_residual_input = (work->u - work->znew).cwiseAbs().maxCoeff();

        // 计算对偶残差
        work->dual_residual_state = (work->v - work->vnew).cwiseAbs().maxCoeff() * cache->rho;
        work->dual_residual_input = (work->z - work->znew).cwiseAbs().maxCoeff() * cache->rho;

        // 检查收敛
        if (work->primal_residual_state < settings->abs_pri_tol &&
            work->primal_residual_input < settings->abs_pri_tol &&
            work->dual_residual_state < settings->abs_dua_tol &&
            work->dual_residual_input < settings->abs_dua_tol) {
            return true;
        }
    }
    return false;
}

int solve(TinySolver* solver)
{
    auto* work = solver->work;
    auto* settings = solver->settings;
    auto* solution = solver->solution;

    // 初始化
    solution->solved = 0;
    solution->iter = 0;
    work->status = 11;  // UNSOLVED
    work->iter = 0;

    // 保存前一次的松弛变量 (用于计算对偶残差)
    tinyMatrix v_prev = work->vnew;
    tinyMatrix z_prev = work->znew;

    for (int i = 0; i < settings->max_iter; ++i) {
        // ADMM 迭代
        backward_pass_grad(solver);
        forward_pass(solver);
        update_slack(solver);
        update_dual(solver);
        update_linear_cost(solver);

        work->iter++;

        // 检查终止条件
        if (termination_condition(solver)) {
            work->status = 1;  // SOLVED

            solution->iter = work->iter;
            solution->solved = 1;
            solution->x = work->vnew;
            solution->u = work->znew;

            return 0;
        }

        // 保存当前松弛变量
        work->v = work->vnew;
        work->z = work->znew;
    }

    // 未收敛，返回当前最优解
    solution->iter = work->iter;
    solution->solved = 0;
    solution->x = work->vnew;
    solution->u = work->znew;

    return 1;
}

}  // namespace tinympc
