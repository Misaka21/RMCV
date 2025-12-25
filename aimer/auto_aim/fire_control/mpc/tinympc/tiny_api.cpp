/**
 * @file tiny_api.cpp
 * @brief TinyMPC 公共 API 实现
 */

#include "tiny_api.hpp"

#include <iostream>

namespace tinympc {

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
    int verbose
)
{
    // 分配内存
    auto* solution = new TinySolution();
    auto* cache = new TinyCache();
    auto* settings = new TinySettings();
    auto* work = new TinyWorkspace();
    auto* solver = new TinySolver();

    solver->solution = solution;
    solver->cache = cache;
    solver->settings = settings;
    solver->work = work;

    *solverp = solver;

    // 初始化 solution
    solution->iter = 0;
    solution->solved = 0;
    solution->x = tinyMatrix::Zero(nx, N);
    solution->u = tinyMatrix::Zero(nu, N - 1);

    // 初始化 workspace
    work->nx = nx;
    work->nu = nu;
    work->N = N;

    work->x = tinyMatrix::Zero(nx, N);
    work->u = tinyMatrix::Zero(nu, N - 1);

    work->q = tinyMatrix::Zero(nx, N);
    work->r = tinyMatrix::Zero(nu, N - 1);

    work->p = tinyMatrix::Zero(nx, N);
    work->d = tinyMatrix::Zero(nu, N - 1);

    work->v = tinyMatrix::Zero(nx, N);
    work->vnew = tinyMatrix::Zero(nx, N);
    work->z = tinyMatrix::Zero(nu, N - 1);
    work->znew = tinyMatrix::Zero(nu, N - 1);

    work->g = tinyMatrix::Zero(nx, N);
    work->y = tinyMatrix::Zero(nu, N - 1);

    // 添加 rho 到 Q, R 对角线
    work->Q = (Q + rho * tinyMatrix::Identity(nx, nx)).diagonal();
    work->R = (R + rho * tinyMatrix::Identity(nu, nu)).diagonal();
    work->Adyn = Adyn;
    work->Bdyn = Bdyn;
    work->fdyn = fdyn;

    work->Xref = tinyMatrix::Zero(nx, N);
    work->Uref = tinyMatrix::Zero(nu, N - 1);

    work->Qu = tinyVector::Zero(nu);

    // 预计算缓存
    int status = tiny_precompute_cache(
        cache, Adyn, Bdyn, fdyn,
        work->Q.asDiagonal(), work->R.asDiagonal(),
        nx, nu, rho, verbose
    );

    return status;
}

int tiny_set_bound_constraints(
    TinySolver* solver,
    const tinyMatrix& x_min,
    const tinyMatrix& x_max,
    const tinyMatrix& u_min,
    const tinyMatrix& u_max
)
{
    if (!solver) return 1;

    solver->work->x_min = x_min;
    solver->work->x_max = x_max;
    solver->work->u_min = u_min;
    solver->work->u_max = u_max;

    return 0;
}

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
    int verbose
)
{
    if (!cache) return 1;

    // 添加 rho 正则化
    tinyMatrix Q1 = Q + rho * tinyMatrix::Identity(nx, nx);
    tinyMatrix R1 = R + rho * tinyMatrix::Identity(nu, nu);

    if (verbose) {
        std::cout << "TinyMPC: Computing Riccati steady-state solution..." << std::endl;
    }

    // Riccati 递推求稳态解
    tinyMatrix Ktp1 = tinyMatrix::Zero(nu, nx);
    tinyMatrix Ptp1 = rho * tinyMatrix::Ones(nx, 1).asDiagonal();
    tinyMatrix Kinf = tinyMatrix::Zero(nu, nx);
    tinyMatrix Pinf = tinyMatrix::Zero(nx, nx);

    for (int i = 0; i < 1000; ++i) {
        // Kinf = (R + B'PB)^-1 * B'PA
        Kinf = (R1 + Bdyn.transpose() * Ptp1 * Bdyn).inverse() *
               Bdyn.transpose() * Ptp1 * Adyn;

        // Pinf = Q + A'P(A - BK)
        Pinf = Q1 + Adyn.transpose() * Ptp1 * (Adyn - Bdyn * Kinf);

        // 检查收敛
        if ((Kinf - Ktp1).cwiseAbs().maxCoeff() < 1e-5) {
            if (verbose) {
                std::cout << "TinyMPC: Riccati converged after " << i + 1 << " iterations" << std::endl;
            }
            break;
        }

        Ktp1 = Kinf;
        Ptp1 = Pinf;
    }

    // 计算缓存矩阵
    tinyMatrix Quu_inv = (R1 + Bdyn.transpose() * Pinf * Bdyn).inverse();
    tinyMatrix AmBKt = (Adyn - Bdyn * Kinf).transpose();
    tinyVector APf = AmBKt * Pinf * fdyn;
    tinyVector BPf = Bdyn.transpose() * Pinf * fdyn;

    cache->rho = rho;
    cache->Kinf = Kinf;
    cache->Pinf = Pinf;
    cache->Quu_inv = Quu_inv;
    cache->AmBKt = AmBKt;
    cache->APf = APf;
    cache->BPf = BPf;

    if (verbose) {
        std::cout << "TinyMPC: Cache precomputed successfully" << std::endl;
    }

    return 0;
}

int tiny_solve(TinySolver* solver)
{
    return solve(solver);
}

int tiny_set_x0(TinySolver* solver, const tinyVector& x0)
{
    if (!solver) return 1;
    solver->work->x.col(0) = x0;
    return 0;
}

int tiny_set_x_ref(TinySolver* solver, const tinyMatrix& x_ref)
{
    if (!solver) return 1;
    solver->work->Xref = x_ref;
    return 0;
}

int tiny_set_u_ref(TinySolver* solver, const tinyMatrix& u_ref)
{
    if (!solver) return 1;
    solver->work->Uref = u_ref;
    return 0;
}

void tiny_cleanup(TinySolver* solver)
{
    if (!solver) return;

    delete solver->solution;
    delete solver->cache;
    delete solver->settings;
    delete solver->work;
    delete solver;
}

}  // namespace tinympc
