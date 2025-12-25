/**
 * @file trajectory_solver.cpp
 * @brief 弹道解算器实现 - 精确空气阻力模型 + 动打动支持
 *
 * 物理模型参考: rm.cv.fans (SJTU) ResistanceFuncLinear
 */

#include "trajectory_solver.hpp"

#include <algorithm>
#include <cmath>

#include <ceres/ceres.h>

namespace autoaim::fire_control {

// ============================================================================
// Ceres 残差函数: 2D 线性阻力模型 (静打动)
// ============================================================================

/**
 * @brief 2D 弹道残差函数 (垂直平面内)
 *
 * 物理推导:
 *   水平: dvx/dt = -k*vx  =>  x(t) = v0*cos(α)/k * (1 - exp(-k*t))
 *   垂直: dvz/dt = -g - k*vz
 *         =>  z(t) = (v0*sin(α) + g/k)/k * (1 - exp(-k*t)) - g*t/k
 *
 * 消去 t，得到 z 关于 x 的隐式方程
 */
class ResistanceFuncLinear2D {
public:
    ResistanceFuncLinear2D(double w, double h, double v0, double g, double k)
        : w_(w), h_(h), v0_(v0), g_(g), k_(k) {}

    template <typename T>
    bool operator()(const T* const pitch, T* residual) const {
        T sin_a = ceres::sin(pitch[0]);
        T cos_a = ceres::cos(pitch[0]);

        if (ceres::abs(cos_a) < T(1e-6)) {
            residual[0] = T(1e6);
            return true;
        }

        T k = T(k_);
        T g = T(g_);
        T v0 = T(v0_);
        T w = T(w_);
        T h = T(h_);

        T ratio = k * w / (v0 * cos_a);
        if (ratio >= T(0.99)) {
            residual[0] = T(1e6);
            return true;
        }

        // 精确弹道方程 (rm.cv.fans)
        T term1 = (k * v0 * sin_a + g) * k * w / (k * k * v0 * cos_a);
        T term2 = g * ceres::log(T(1.0) - ratio) / (k * k);
        residual[0] = term1 + term2 - h;
        return true;
    }

private:
    double w_, h_, v0_, g_, k_;
};

// ============================================================================
// Ceres 残差函数: 3D 线性阻力模型 (动打动)
// ============================================================================

class ResistanceFuncLinear3D {
public:
    ResistanceFuncLinear3D(
        const Eigen::Vector3d& target, double v0,
        const Eigen::Vector3d& vehicle_vel, double g, double k)
        : target_(target), v0_(v0), vehicle_vel_(vehicle_vel), g_(g), k_(k) {}

    template <typename T>
    bool operator()(const T* const angles, T* residual) const {
        T yaw = angles[0], pitch = angles[1];
        T cos_p = ceres::cos(pitch), sin_p = ceres::sin(pitch);
        T cos_y = ceres::cos(yaw), sin_y = ceres::sin(yaw);

        T v0 = T(v0_), k = T(k_), g = T(g_);

        // 初始速度 = 弹速 + 车辆速度
        T vx0 = v0 * cos_p * cos_y + T(vehicle_vel_.x());
        T vy0 = v0 * cos_p * sin_y + T(vehicle_vel_.y());
        T vz0 = v0 * sin_p + T(vehicle_vel_.z());

        T v_horiz = ceres::sqrt(vx0 * vx0 + vy0 * vy0);
        if (v_horiz < T(0.1)) {
            residual[0] = residual[1] = residual[2] = T(1e6);
            return true;
        }

        T target_horiz = T(std::hypot(target_.x(), target_.y()));
        T t = target_horiz / v_horiz * (T(1.0) + k * target_horiz / (T(2.0) * v_horiz));

        if (t < T(0.001) || t > T(5.0)) {
            residual[0] = residual[1] = residual[2] = T(1e6);
            return true;
        }

        T exp_neg_kt = ceres::exp(-k * t);
        T one_minus_exp = T(1.0) - exp_neg_kt;

        residual[0] = vx0 / k * one_minus_exp - T(target_.x());
        residual[1] = vy0 / k * one_minus_exp - T(target_.y());
        residual[2] = (vz0 + g / k) / k * one_minus_exp - g * t / k - T(target_.z());
        return true;
    }

private:
    Eigen::Vector3d target_, vehicle_vel_;
    double v0_, g_, k_;
};

// ============================================================================
// TrajectorySolver 实现
// ============================================================================

TrajectorySolver::TrajectorySolver(const TrajectoryConfig& config) : config_(config) {}

AimResult TrajectorySolver::solve(
    const Eigen::Vector3d& target_pos,
    double bullet_speed,
    const Eigen::Vector3d& vehicle_velocity) const
{
    TrajectoryInput input;
    input.target_pos = target_pos;
    input.bullet_speed = bullet_speed;
    input.vehicle_velocity = vehicle_velocity;
    return solve(input);
}

AimResult TrajectorySolver::solve(const TrajectoryInput& input) const {
    // 根据车辆速度自动选择求解方式
    if (input.vehicle_velocity.norm() > 0.1) {
        return solve_3d(input);  // 动打动
    }
    return solve_2d(input);  // 静打动 (更快)
}

AimResult TrajectorySolver::solve_2d(const TrajectoryInput& input) const {
    AimResult result;
    const auto& target = input.target_pos;
    double v0 = input.bullet_speed;
    double horiz_dist = horizontal_distance(target);

    if (horiz_dist < 0.1 || v0 < 5.0) return result;

    result.yaw = std::atan2(target.y(), target.x());
    result.distance = target.norm();

    double pitch = std::atan2(target.z(), horiz_dist);

    ceres::Problem problem;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ResistanceFuncLinear2D, 1, 1>(
            new ResistanceFuncLinear2D(horiz_dist, target.z(), v0, config_.g, config_.resistance_k)),
        nullptr, &pitch);

    problem.SetParameterLowerBound(&pitch, 0, -M_PI / 3);
    problem.SetParameterUpperBound(&pitch, 0, M_PI / 3);

    ceres::Solver::Options options;
    options.max_num_iterations = config_.max_iterations;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = config_.verbose;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    result.valid = true;
    result.pitch = pitch;
    result.fly_time = estimate_fly_time(horiz_dist / std::cos(pitch), v0);
    result.hit_point = target;
    return result;
}

AimResult TrajectorySolver::solve_3d(const TrajectoryInput& input) const {
    AimResult result;
    const auto& target = input.target_pos;
    double v0 = input.bullet_speed;

    if (target.norm() < 0.1 || v0 < 5.0) return result;

    double angles[2] = {
        std::atan2(target.y(), target.x()),
        std::atan2(target.z(), horizontal_distance(target))
    };

    ceres::Problem problem;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ResistanceFuncLinear3D, 3, 2>(
            new ResistanceFuncLinear3D(target, v0, input.vehicle_velocity, config_.g, config_.resistance_k)),
        nullptr, angles);

    problem.SetParameterLowerBound(angles, 0, -M_PI);
    problem.SetParameterUpperBound(angles, 0, M_PI);
    problem.SetParameterLowerBound(angles, 1, -M_PI / 3);
    problem.SetParameterUpperBound(angles, 1, M_PI / 3);

    ceres::Solver::Options options;
    options.max_num_iterations = config_.max_iterations;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    result.valid = true;
    result.yaw = angles[0];
    result.pitch = angles[1];
    result.distance = target.norm();
    result.fly_time = estimate_fly_time(result.distance, v0);
    result.hit_point = target;
    return result;
}

double TrajectorySolver::estimate_fly_time(double distance, double bullet_speed) const {
    if (bullet_speed < 1.0) return 0;
    double t = distance / bullet_speed;
    return t * (1.0 + config_.resistance_k * distance / (2.0 * bullet_speed));
}

Eigen::Vector3d TrajectorySolver::compute_hit_point(
    double yaw, double pitch, double bullet_speed,
    const Eigen::Vector3d& vehicle_velocity, double fly_time) const
{
    double k = config_.resistance_k, g = config_.g;
    double vx0 = bullet_speed * std::cos(pitch) * std::cos(yaw) + vehicle_velocity.x();
    double vy0 = bullet_speed * std::cos(pitch) * std::sin(yaw) + vehicle_velocity.y();
    double vz0 = bullet_speed * std::sin(pitch) + vehicle_velocity.z();

    double exp_neg_kt = std::exp(-k * fly_time);
    double one_minus_exp = 1.0 - exp_neg_kt;

    return Eigen::Vector3d(
        vx0 / k * one_minus_exp,
        vy0 / k * one_minus_exp,
        (vz0 + g / k) / k * one_minus_exp - g * fly_time / k
    );
}

}  // namespace autoaim::fire_control
