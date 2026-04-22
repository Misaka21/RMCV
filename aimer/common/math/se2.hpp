/**
 * @file se2.hpp
 * @brief 轻量级 SE(2) 实现（用于 InEKF）
 *
 * SE(2): 2D 刚体变换群
 *   [ R(theta)  t ]
 *   [    0      1 ]
 *
 * 李代数 se(2) ≅ R^3, 元素记作 xi = [xi_theta, xi_x, xi_y]^T
 */

#ifndef AIMER_COMMON_MATH_SE2_HPP
#define AIMER_COMMON_MATH_SE2_HPP

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aimer::math {

struct SE2 {
    double theta = 0.0;
    Eigen::Vector2d t;

    SE2() = default;
    SE2(double theta_, const Eigen::Vector2d& t_) : theta(theta_), t(t_) {}

    static SE2 Identity() { return SE2(0.0, Eigen::Vector2d::Zero()); }

    Eigen::Matrix3d matrix() const {
        Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
        double c = std::cos(theta), s = std::sin(theta);
        M(0, 0) = c;  M(0, 1) = -s;  M(0, 2) = t.x();
        M(1, 0) = s;  M(1, 1) =  c;  M(1, 2) = t.y();
        return M;
    }

    static SE2 fromMatrix(const Eigen::Matrix3d& M) {
        SE2 X;
        X.theta = std::atan2(M(1, 0), M(0, 0));
        X.t = M.block<2, 1>(0, 2);
        return X;
    }

    SE2 operator*(const SE2& other) const {
        return SE2(theta + other.theta,
                   t + Eigen::Rotation2Dd(theta) * other.t);
    }

    SE2 inverse() const {
        double nt = -theta;
        return SE2(nt, -Eigen::Rotation2Dd(nt) * t);
    }

    static SE2 exp(const Eigen::Vector3d& xi) {
        double xi_theta = xi(0);
        double c = std::cos(xi_theta), s = std::sin(xi_theta);
        double V00, V01, V10, V11;
        if (std::abs(xi_theta) < 1e-8) {
            V00 = 1.0;            V01 = -0.5 * xi_theta;
            V10 = 0.5 * xi_theta;  V11 = 1.0;
        } else {
            V00 = s / xi_theta;            V01 = (c - 1.0) / xi_theta;
            V10 = (1.0 - c) / xi_theta;    V11 = s / xi_theta;
        }
        return SE2(xi_theta,
                   Eigen::Vector2d(V00 * xi(1) + V01 * xi(2),
                                   V10 * xi(1) + V11 * xi(2)));
    }

    Eigen::Vector3d log() const {
        Eigen::Vector3d xi;
        xi(0) = theta;
        double c = std::cos(theta), s = std::sin(theta);
        double W00, W01, W10, W11;
        if (std::abs(theta) < 1e-8) {
            W00 = 1.0;            W01 = 0.5 * theta;
            W10 = -0.5 * theta;   W11 = 1.0;
        } else {
            W00 = theta * s / (2.0 * (1.0 - c));  W01 = -0.5 * theta;
            W10 = 0.5 * theta;                    W11 = theta * s / (2.0 * (1.0 - c));
        }
        xi(1) = W00 * t.x() + W01 * t.y();
        xi(2) = W10 * t.x() + W11 * t.y();
        return xi;
    }

    Eigen::Matrix3d Adj() const {
        Eigen::Matrix3d Ad = Eigen::Matrix3d::Zero();
        double c = std::cos(theta), s = std::sin(theta);
        Ad(0, 0) = 1.0;
        Ad(1, 0) = t.y();  Ad(1, 1) = c;  Ad(1, 2) = -s;
        Ad(2, 0) = -t.x(); Ad(2, 1) = s;  Ad(2, 2) = c;
        return Ad;
    }

    Eigen::Vector2d act(const Eigen::Vector2d& p) const {
        return Eigen::Rotation2Dd(theta) * p + t;
    }
};

inline Eigen::Matrix3d se2_hat(const Eigen::Vector3d& xi) {
    Eigen::Matrix3d Xi = Eigen::Matrix3d::Zero();
    Xi(0, 1) = -xi(0);  Xi(0, 2) = xi(1);
    Xi(1, 0) =  xi(0);  Xi(1, 2) = xi(2);
    return Xi;
}

inline Eigen::Vector3d se2_vee(const Eigen::Matrix3d& Xi) {
    return Eigen::Vector3d(Xi(1, 0), Xi(0, 2), Xi(1, 2));
}

} // namespace aimer::math

#endif // AIMER_COMMON_MATH_SE2_HPP
