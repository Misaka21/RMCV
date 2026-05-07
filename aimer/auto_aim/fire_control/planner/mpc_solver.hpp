/**
 * @file mpc_solver.hpp
 * @brief 双积分器 ADMM MPC 求解器 (简化自 TinyMPC)
 *
 * 固定模型: 双积分器 (位置+速度状态, 加速度控制)
 *   A = [[1, dt],    B = [[0],
 *        [0, 1]]          [dt]]
 *
 * 仅支持输入盒约束 (加速度上限), 无状态约束/SOC/线性约束/自适应 rho
 */

#pragma once

#include <Eigen/Dense>

namespace autoaim::fire_control {

class DoubleIntegratorMPC {
public:
    /// @param dt      控制周期 (s)
    /// @param horizon 预测步数
    /// @param q_pos   位置跟踪权重
    /// @param q_vel   速度跟踪权重
    /// @param r       控制代价权重
    /// @param rho     ADMM 惩罚因子
    DoubleIntegratorMPC(double dt, int horizon,
                        double q_pos, double q_vel, double r, double rho);

    /// 设置参考轨迹 Xref (2×N) 和 Uref (1×N-1, 默认为零)
    void set_reference(const Eigen::Ref<const Eigen::Matrix2Xd>& Xref);

    /// 设置输入盒约束 u_min, u_max (1×N-1)
    void set_input_bounds(const Eigen::Ref<const Eigen::Matrix<double, 1, Eigen::Dynamic>>& u_min,
                          const Eigen::Ref<const Eigen::Matrix<double, 1, Eigen::Dynamic>>& u_max);

    /// 设置初始状态 x0 = [position, velocity]
    void set_initial_state(const Eigen::Vector2d& x0);

    /// ADMM 迭代求解, 返回实际迭代次数. warm-start: 调用前 x_, u_ 保留上次解
    int solve(int max_iter);

    /// 获取第 k 步状态 [pos, vel]
    Eigen::Vector2d state_at(int k) const;
    /// 获取第 k 步控制 [acc]
    double control_at(int k) const;

    int horizon() const { return N_; }
    int last_iter() const { return iter_; }

    const Eigen::Matrix2Xd& state_trajectory() const { return x_; }
    const Eigen::Matrix<double, 1, Eigen::Dynamic>& control_trajectory() const { return u_; }

private:
    void backward_pass_grad();
    void forward_pass();
    void update_slack();
    void update_dual();
    void update_linear_cost();

    double dt_;
    int N_;
    double rho_;

    // 模型矩阵
    Eigen::Matrix2d A_;
    Eigen::Vector2d B_;  // 2×1

    // 增广成本: Q_aug = Q + rho*I, R_aug = R + rho*I
    Eigen::DiagonalMatrix<double, 2> Q_aug_;
    double R_aug_;

    // ==== 预计算缓存 (Riccati 收敛解) ====
    Eigen::Matrix<double, 1, 2> Kinf_;  // 1×2, 无限时域 LQR 增益
    Eigen::Matrix2d Pinf_;              // 2×2, 无限时域代价-to-go
    double Quu_inv_;                    // (R + B'PB)^-1
    Eigen::Matrix2d AmBKt_;             // (A - BK)^T

    // ==== 工作区 (按 horizon 动态分配) ====
    Eigen::Matrix2Xd Xref_;                          // 2×N 状态参考
    Eigen::Matrix<double, 1, Eigen::Dynamic> Uref_;  // 1×(N-1) 控制参考

    Eigen::Matrix2Xd x_;     // 2×N  状态轨迹
    Eigen::Matrix<double, 1, Eigen::Dynamic> u_;    // 1×(N-1) 控制轨迹

    Eigen::Matrix2Xd q_, p_;                         // 2×N  线性代价, Riccati 项
    Eigen::Matrix<double, 1, Eigen::Dynamic> r_, d_; // 1×(N-1)

    Eigen::Matrix2Xd v_, vnew_, g_;                  // 2×N  状态松弛+对偶
    Eigen::Matrix<double, 1, Eigen::Dynamic> z_, znew_, y_;  // 1×(N-1) 输入松弛+对偶
    Eigen::Matrix<double, 1, Eigen::Dynamic> u_min_, u_max_;

    int iter_ = 0;
};

// ==================== 实现 ====================

inline DoubleIntegratorMPC::DoubleIntegratorMPC(
    double dt, int horizon, double q_pos, double q_vel, double r, double rho)
    : dt_(dt), N_(horizon), rho_(rho)
{
    // 模型矩阵
    A_ << 1.0, dt_, 0.0, 1.0;
    B_ << 0.0, dt_;

    // 增广成本 Q_aug = Q + rho*I, R_aug = R + rho*I
    Q_aug_.diagonal() << q_pos + rho, q_vel + rho;
    R_aug_ = r + rho;

    // 分配工作区
    const int N = N_;
    Xref_  = Eigen::Matrix2Xd::Zero(2, N);
    Uref_  = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, N - 1);
    x_     = Eigen::Matrix2Xd::Zero(2, N);
    u_     = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, N - 1);
    q_     = Eigen::Matrix2Xd::Zero(2, N);
    p_     = Eigen::Matrix2Xd::Zero(2, N);
    r_     = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, N - 1);
    d_     = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, N - 1);
    v_     = Eigen::Matrix2Xd::Zero(2, N);
    vnew_  = Eigen::Matrix2Xd::Zero(2, N);
    g_     = Eigen::Matrix2Xd::Zero(2, N);
    z_     = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, N - 1);
    znew_  = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, N - 1);
    y_     = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1, N - 1);
    u_min_ = Eigen::Matrix<double, 1, Eigen::Dynamic>::Constant(1, N - 1, -1e17);
    u_max_ = Eigen::Matrix<double, 1, Eigen::Dynamic>::Constant(1, N - 1, 1e17);

    // Riccati 迭代求 Kinf, Pinf
    {
        Eigen::Matrix<double, 1, 2> Ktp1 = Eigen::Matrix<double, 1, 2>::Zero();
        Eigen::Matrix2d Ptp1 = rho * Eigen::Matrix2d::Identity();

        for (int i = 0; i < 1000; ++i) {
            // S = R_aug + B' * P * B  (scalar for nu=1)
            double S = R_aug_ + B_.transpose() * Ptp1 * B_;
            double S_inv = 1.0 / S;

            // K = S^-1 * B' * P * A
            Kinf_ = S_inv * (B_.transpose() * Ptp1 * A_);

            // P = Q_aug + A' * P * (A - B * K)
            Pinf_ = Q_aug_.toDenseMatrix()
                + A_.transpose() * Ptp1 * (A_ - B_ * Kinf_);

            if ((Kinf_ - Ktp1).cwiseAbs().maxCoeff() < 1e-5) break;
            Ktp1 = Kinf_;
            Ptp1 = Pinf_;
        }
    }

    // 预计算缓存
    double S = R_aug_ + B_.transpose() * Pinf_ * B_;
    Quu_inv_ = 1.0 / S;
    AmBKt_ = (A_ - B_ * Kinf_).transpose();
}

inline void DoubleIntegratorMPC::set_reference(
    const Eigen::Ref<const Eigen::Matrix2Xd>& Xref)
{
    Xref_ = Xref;
    Uref_.setZero(1, N_ - 1);
}

inline void DoubleIntegratorMPC::set_input_bounds(
    const Eigen::Ref<const Eigen::Matrix<double, 1, Eigen::Dynamic>>& u_min,
    const Eigen::Ref<const Eigen::Matrix<double, 1, Eigen::Dynamic>>& u_max)
{
    u_min_ = u_min;
    u_max_ = u_max;
}

inline void DoubleIntegratorMPC::set_initial_state(const Eigen::Vector2d& x0) {
    x_.col(0) = x0;
}

inline int DoubleIntegratorMPC::solve(int max_iter) {
    iter_ = 0;

    // 首次调用: 用参考初始化轨迹
    if (u_.squaredNorm() < 1e-30) {
        x_ = Xref_;
        v_ = x_;
        g_.setZero();
        z_.setZero();
        y_.setZero();
    }

    for (int i = 0; i < max_iter; ++i, ++iter_) {
        backward_pass_grad();
        forward_pass();
        update_slack();
        update_dual();
        update_linear_cost();

        // 收敛检查 (每 4 次迭代检查一次)
        if (i % 4 == 3) {
            double pri_state = (x_ - vnew_).cwiseAbs().maxCoeff();
            double pri_input = (u_ - znew_).cwiseAbs().maxCoeff();
            double dua_state = (v_ - vnew_).cwiseAbs().maxCoeff() * rho_;
            double dua_input = (z_ - znew_).cwiseAbs().maxCoeff() * rho_;
            if (pri_state < 1e-3 && pri_input < 1e-3
                && dua_state < 1e-3 && dua_input < 1e-3)
            {
                v_ = vnew_;
                z_ = znew_;
                break;
            }
        }

        v_ = vnew_;
        z_ = znew_;
    }

    return iter_;
}

inline Eigen::Vector2d DoubleIntegratorMPC::state_at(int k) const {
    return vnew_.col(k);  // 返回收敛后的松弛变量 = 原始变量
}

inline double DoubleIntegratorMPC::control_at(int k) const {
    return znew_(0, k);
}

// ==================== ADMM 子步骤 ====================

inline void DoubleIntegratorMPC::backward_pass_grad() {
    // p_{N-1} 已在 update_linear_cost 中设置
    for (int i = N_ - 2; i >= 0; --i) {
        // d_i = Quu_inv * (B' * p_{i+1} + r_i)
        d_(0, i) = Quu_inv_ * (B_.dot(p_.col(i + 1)) + r_(0, i));
        // p_i = q_i + AmBKt * p_{i+1} - Kinf' * r_i
        p_.col(i) = q_.col(i)
            + AmBKt_ * p_.col(i + 1)
            - Kinf_.transpose() * r_(0, i);
    }
}

inline void DoubleIntegratorMPC::forward_pass() {
    for (int i = 0; i < N_ - 1; ++i) {
        // u_i = -Kinf * x_i - d_i
        u_(0, i) = -(Kinf_ * x_.col(i))(0) - d_(0, i);
        // x_{i+1} = A * x_i + B * u_i
        x_.col(i + 1) = A_ * x_.col(i) + B_ * u_(0, i);
    }
}

inline void DoubleIntegratorMPC::update_slack() {
    // vnew = x + g, 然后投影到盒约束 (本题无状态盒约束, 跳过)
    vnew_ = x_ + g_;

    // znew = u + y, 投影到 [u_min, u_max]
    znew_ = (u_ + y_).cwiseMax(u_min_).cwiseMin(u_max_);
}

inline void DoubleIntegratorMPC::update_dual() {
    g_ += x_ - vnew_;
    y_ += u_ - znew_;
}

inline void DoubleIntegratorMPC::update_linear_cost() {
    // q = -Q * Xref - rho*(vnew - g)
    q_.row(0) = -Q_aug_.diagonal()(0) * Xref_.row(0).array()
        - rho_ * (vnew_.row(0) - g_.row(0)).array();
    q_.row(1) = -Q_aug_.diagonal()(1) * Xref_.row(1).array()
        - rho_ * (vnew_.row(1) - g_.row(1)).array();

    // r = -R * Uref - rho*(znew - y)
    r_ = -R_aug_ * Uref_
        - rho_ * (znew_ - y_);

    // 终端代价: p_{N-1} = -Pinf * Xref_{N-1} - rho*(vnew_{N-1} - g_{N-1})
    p_.col(N_ - 1) = -Pinf_ * Xref_.col(N_ - 1)
        - rho_ * (vnew_.col(N_ - 1) - g_.col(N_ - 1));
}

}  // namespace autoaim::fire_control
