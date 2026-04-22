/**
 * @file invariant_ekf.hpp
 * @brief 基于 SE(2) 的左不变扩展卡尔曼滤波器
 *
 * 设计目标: 替换标准 EKF，用于平面底盘状态估计。
 *
 * 状态分解:
 *   - 群状态 X ∈ SE(2): (x_c, y_c, theta)
 *   - 向量状态 b ∈ R^VEC_DIM: [z_c, v_x, v_y, v_z, omega, r, l, h, ...]
 *
 * 误差定义 (右不变误差):
 *   η^R = X̂ · X^{-1}  ⇔  X̂ = exp(ξ^∧) · X
 *   ξ ∈ se(2) ≅ R^3: [xi_theta, xi_x, xi_y]^T
 *
 * 预测 (group-affine CV 模型):
 *   u = [omega, v_x, v_y]^T (世界坐标系)
 *   X_{k+1} = exp(u·dt) · X_k
 *   b_{k+1} = b_k
 *
 * 误差状态转移 (自治):
 *   F = [ Ad_{exp(-u·dt)}   F_12
 *         0                 I       ]
 *   其中 F_12 将 [delta_vx, delta_vy, delta_omega] 映射到李代数误差增量。
 *
 * 观测更新:
 *   观测函数接收增广状态 y[0:TOTAL_DIM]，其中 y[0:3]=ξ, y[3:]=b。
 *   在名义点 ξ=0 处线性化，得到关于 [ξ; delta_b] 的雅可比 H。
 *   更新: X ← exp(K_ξ · innov) · X,   b ← b + K_b · innov
 */

#ifndef AIMER_COMMON_FILTER_INVARIANT_EKF_HPP
#define AIMER_COMMON_FILTER_INVARIANT_EKF_HPP

#include <algorithm>
#include <cmath>

#include <ceres/jet.h>
#include <Eigen/Dense>

#include "aimer/common/math/se2.hpp"

namespace aimer::filter {

enum class InEKFUpdateStatus {
    ACCEPTED,
    REJECTED,
    RESET
};

/**
 * @tparam VEC_DIM 向量状态维度 (b 的维度)
 * @tparam MEAS_DIM 观测维度
 */
template<int VEC_DIM, int MEAS_DIM>
class InvariantEkf {
public:
    static constexpr int TOTAL_DIM = 3 + VEC_DIM;

    using VectorX   = Eigen::Matrix<double, TOTAL_DIM, 1>;
    using VectorB   = Eigen::Matrix<double, VEC_DIM, 1>;
    using VectorZ   = Eigen::Matrix<double, MEAS_DIM, 1>;
    using MatrixXX  = Eigen::Matrix<double, TOTAL_DIM, TOTAL_DIM>;
    using MatrixZZ  = Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>;
    using MatrixZX  = Eigen::Matrix<double, MEAS_DIM, TOTAL_DIM>;
    using MatrixXZ  = Eigen::Matrix<double, TOTAL_DIM, MEAS_DIM>;

    struct PredictResult {
        VectorX x_p;
        MatrixXX F;
    };

    struct MeasureResult {
        VectorZ z_e;
        MatrixZX H;
    };

private:
    aimer::math::SE2 X_;   ///< 群状态 (xc, yc, theta)
    VectorB b_;            ///< 向量状态
    MatrixXX P_;           ///< 误差协方差 (关于 [ξ; delta_b])

    int reject_count_ = 0;
    double q_scale_ = 1.0;

    static constexpr double INF = 1e9;

public:
    InvariantEkf() : X_(aimer::math::SE2::Identity()), b_(VectorB::Zero()), P_(MatrixXX::Identity() * INF) {}

    void init(const aimer::math::SE2& X0, const VectorB& b0, const MatrixXX& P0 = MatrixXX::Identity()) {
        X_ = X0;
        b_ = b0;
        P_ = P0;
        reset_gating_state();
    }

    aimer::math::SE2 get_X() const { return X_; }
    VectorB get_b() const { return b_; }
    MatrixXX get_P() const { return P_; }
    double get_q_scale() const { return q_scale_; }
    int get_reject_count() const { return reject_count_; }

    void set_X(const aimer::math::SE2& X) { X_ = X; }
    void set_b(const VectorB& b) { b_ = b; }

    void reset_gating_state() {
        reject_count_ = 0;
        q_scale_ = 1.0;
    }

    // ========================================================================
    // 预测步骤
    // ========================================================================

    /**
     * @brief 左不变预测
     * @param u 输入 [omega, vx, vy]^T (世界坐标系, rad/s, m/s)
     * @param dt 时间步长
     * @param Q_base 基础过程噪声 (在李代数+向量空间上)
     */
    void predict(const Eigen::Vector3d& u, double dt, const MatrixXX& Q_base) {
        const double omega = u(0);
        const double vx = u(1);
        const double vy = u(2);

        // 1. 名义群状态传播: X ← exp(u·dt) · X
        X_ = aimer::math::SE2::exp(Eigen::Vector3d(omega * dt, vx * dt, vy * dt)) * X_;

        // 2. 向量状态不变 (CV 模型)
        // b_ 不变

        // 3. 误差状态转移矩阵 F
        MatrixXX F = MatrixXX::Identity();

        // F_11 = Ad_{exp(-u·dt)}
        Eigen::Matrix3d Ad_exp_neg = aimer::math::SE2::exp(Eigen::Vector3d(-omega * dt, -vx * dt, -vy * dt)).Adj();
        F.block<3, 3>(0, 0) = Ad_exp_neg;

        // F_12: [delta_omega, delta_vx, delta_vy] -> [xi_theta, xi_x, xi_y] 增量
        // 向量状态 b 的索引假设: [z_c, v_x, v_y, v_z, omega, r, l, h, ...]
        // 这里我们假设 omega 在 b 的第 4 个位置 (索引 4)，vx 在 1，vy 在 2。
        // 注意：F_12 不强制状态顺序，留给外部调用者通过 Q 矩阵的结构来处理噪声注入。
        // 这里只保留最通用的自治误差动力学：速度/角速度误差直接驱动李代数误差。
        F(0, 4) = dt;   // delta_omega -> xi_theta
        F(1, 1) = dt;   // delta_vx    -> xi_x
        F(2, 2) = dt;   // delta_vy    -> xi_y

        // 4. 过程噪声转换: 左不变误差噪声 = Ad_{X^{-1}} · w_world
        // 将 Q 的左上角 3x3 (李代数部分) 转换到当前车体坐标系
        MatrixXX Q = Q_base;
        Eigen::Matrix3d Ad_inv = X_.inverse().Adj();
        Q.block<3, 3>(0, 0) = Ad_inv * Q_base.block<3, 3>(0, 0) * Ad_inv.transpose();
        Q *= q_scale_;

        // 5. 协方差传播
        P_ = F * P_ * F.transpose() + Q;
    }

    // ========================================================================
    // 观测步骤 (Ceres Jet 自动微分)
    // ========================================================================

    /**
     * @brief 计算观测结果（不修改内部状态）
     *
     * 输入约定: x[0:3] = 李代数误差 ξ, x[3:TOTAL_DIM] = 向量状态 b
     * 函数内部应使用 exp(ξ) · X_nominal 构造群状态。
     *
     * 示例:
     * @code
     * struct BodyMeasure {
     *     double xc, yc, theta; // 名义群状态
     *     int armor_id;
     *     template<typename T>
     *     void operator()(const T x[TOTAL_DIM], T z[MEAS_DIM]) const {
     *         // 构造扰动后的 SE2
     *         T xi_t = x[0], xi_x = x[1], xi_y = x[2];
     *         T ct = ceres::cos(xi_t), st = ceres::sin(xi_t);
     *         // V 矩阵...
     *         T dx = V00*xi_x + V01*xi_y;
     *         T dy = V10*xi_x + V11*xi_y;
     *         T theta_p = theta + xi_t;
     *         T c0 = ceres::cos(theta), s0 = ceres::sin(theta);
     *         T xc_p = xc + c0*dx - s0*dy;
     *         T yc_p = yc + s0*dx + c0*dy;
     *         // ... 用向量状态 x[3:] 计算观测
     *     }
     * };
     * @endcode
     */
    template<typename MeasureFunc>
    MeasureResult measure(MeasureFunc&& func) const {
        ceres::Jet<double, TOTAL_DIM> x_jet[TOTAL_DIM];
        for (int i = 0; i < TOTAL_DIM; ++i) {
            x_jet[i].a = (i < 3) ? 0.0 : b_[i - 3];  // ξ=0, b=b_nom
            x_jet[i].v.setZero();
            x_jet[i].v[i] = 1.0;
        }

        ceres::Jet<double, TOTAL_DIM> z_jet[MEAS_DIM];
        func(x_jet, z_jet);

        MeasureResult result;
        for (int i = 0; i < MEAS_DIM; ++i) {
            result.z_e[i] = z_jet[i].a;
            result.H.row(i) = z_jet[i].v.transpose();
        }
        return result;
    }

    template<typename MeasureFunc>
    void update_forward(MeasureFunc&& func, const VectorZ& y, const MatrixZZ& R) {
        MeasureResult res = measure(func);
        MatrixZZ S = res.H * P_ * res.H.transpose() + R;
        MatrixXZ K = P_ * res.H.transpose() * S.inverse();
        VectorX delta = K * (y - res.z_e);

        // 群状态修正: X ← exp(ξ) · X
        X_ = aimer::math::SE2::exp(delta.head<3>()) * X_;
        // 向量状态修正
        b_ += delta.tail<VEC_DIM>();
        // 协方差修正
        P_ = (MatrixXX::Identity() - K * res.H) * P_;
    }

    template<typename MeasureFunc>
    InEKFUpdateStatus update_forward_gated(
        MeasureFunc&& func,
        const VectorZ& y,
        const MatrixZZ& R,
        const aimer::math::SE2& reset_X,
        const VectorB& reset_b,
        double chi2_threshold = 9.21,
        int max_reject = 5,
        double q_scale_increase = 1.5,
        double q_scale_decay = 0.9
    ) {
        MeasureResult res = measure(func);
        MatrixZZ S = res.H * P_ * res.H.transpose() + R;
        VectorZ innovation = y - res.z_e;
        double mahalanobis_sq = innovation.transpose() * S.inverse() * innovation;

        double threshold = chi2_threshold;
        if (q_scale_ > 1.0) threshold *= q_scale_;

        if (mahalanobis_sq > threshold) {
            reject_count_++;
            q_scale_ *= q_scale_increase;
            if (reject_count_ >= max_reject) {
                X_ = reset_X;
                b_ = reset_b;
                P_ = MatrixXX::Identity();
                reject_count_ = 0;
                q_scale_ = 1.0;
                return InEKFUpdateStatus::RESET;
            }
            return InEKFUpdateStatus::REJECTED;
        }

        reject_count_ = 0;
        q_scale_ = std::max(1.0, q_scale_ * q_scale_decay);

        MatrixXZ K = P_ * res.H.transpose() * S.inverse();
        VectorX delta = K * innovation;

        X_ = aimer::math::SE2::exp(delta.head<3>()) * X_;
        b_ += delta.tail<VEC_DIM>();
        P_ = (MatrixXX::Identity() - K * res.H) * P_;

        return InEKFUpdateStatus::ACCEPTED;
    }
};

} // namespace aimer::filter

#endif // AIMER_COMMON_FILTER_INVARIANT_EKF_HPP
