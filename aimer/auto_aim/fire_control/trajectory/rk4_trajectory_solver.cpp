/**
 * @file rk4_trajectory_solver.cpp
 * @brief 基于四阶龙格-库塔 (RK4) 的弹道求解器实现
 */

#include "rk4_trajectory_solver.hpp"

#include <algorithm>
#include <cmath>

#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

// ============================================================================
// 内部参数结构 (避免多次读取配置)
// ============================================================================

struct Rk4Params {
    DragParams drag;
    double dt = 0.001;
    int max_iter = 50;
    double tolerance = 0.001;
    double max_fly_time = 3.0;

    static Rk4Params from_config() {
        Rk4Params p;

        // 阻力模型
        std::string model_str = runtime_param::get_param<std::string>(
            "AutoAim.FireControl.Trajectory.drag_model");
        p.drag.model = (model_str == "linear") ? DragModel::LINEAR : DragModel::QUADRATIC;
        p.drag.k = runtime_param::get_param<double>("AutoAim.FireControl.Trajectory.air_resistance_k");
        p.drag.g = runtime_param::get_param<double>("AutoAim.FireControl.Trajectory.gravity");

        // RK4参数
        p.dt = runtime_param::get_param<double>("AutoAim.FireControl.Trajectory.rk4_dt");
        p.max_iter = static_cast<int>(runtime_param::get_param<int64_t>("AutoAim.FireControl.Trajectory.rk4_max_iter"));
        p.tolerance = runtime_param::get_param<double>("AutoAim.FireControl.Trajectory.rk4_tolerance");

        return p;
    }
};

// ============================================================================
// RK4 核心算法
// ============================================================================

TrajectoryState Rk4TrajectorySolver::derivative(
    const TrajectoryState& state,
    const DragParams& params) const
{
    // state: [x, y, z, vx, vy, vz]
    double vx = state[3];
    double vy = state[4];
    double vz = state[5];

    double ax, ay, az;

    if (params.model == DragModel::LINEAR) {
        // 线性阻力: a = -k * v
        ax = -params.k * vx;
        ay = -params.k * vy;
        az = -params.g - params.k * vz;
    } else {
        // 二次阻力: a = -k * v * |v|
        double v_mag = std::sqrt(vx * vx + vy * vy + vz * vz);
        ax = -params.k * vx * v_mag;
        ay = -params.k * vy * v_mag;
        az = -params.g - params.k * vz * v_mag;
    }

    return {vx, vy, vz, ax, ay, az};
}

TrajectoryState Rk4TrajectorySolver::rk4_step(
    const TrajectoryState& state,
    double dt,
    const DragParams& params) const
{
    // RK4: y_{n+1} = y_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    auto k1 = derivative(state, params);

    TrajectoryState state2;
    for (int i = 0; i < 6; ++i) {
        state2[i] = state[i] + 0.5 * dt * k1[i];
    }
    auto k2 = derivative(state2, params);

    TrajectoryState state3;
    for (int i = 0; i < 6; ++i) {
        state3[i] = state[i] + 0.5 * dt * k2[i];
    }
    auto k3 = derivative(state3, params);

    TrajectoryState state4;
    for (int i = 0; i < 6; ++i) {
        state4[i] = state[i] + dt * k3[i];
    }
    auto k4 = derivative(state4, params);

    // 合并
    TrajectoryState next;
    for (int i = 0; i < 6; ++i) {
        next[i] = state[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    return next;
}

TrajectoryResult Rk4TrajectorySolver::integrate(
    const TrajectoryState& initial_state,
    double target_distance,
    const DragParams& params) const
{
    TrajectoryResult result;
    result.hit = false;

    TrajectoryState state = initial_state;
    double t = 0;

    // 积分直到水平距离超过目标或超时
    while (t < max_fly_time_) {
        double horiz_dist = std::hypot(state[0], state[1]);

        // 到达目标水平距离
        if (horiz_dist >= target_distance) {
            result.hit = true;
            result.fly_time = t;
            result.hit_point = Eigen::Vector3d(state[0], state[1], state[2]);
            result.hit_velocity = Eigen::Vector3d(state[3], state[4], state[5]);
            return result;
        }

        // 落地检测 (z < -10m 视为落地)
        if (state[2] < -10.0) {
            break;
        }

        state = rk4_step(state, dt_, params);
        t += dt_;
    }

    // 未命中，返回最终状态
    result.fly_time = t;
    result.hit_point = Eigen::Vector3d(state[0], state[1], state[2]);
    result.hit_velocity = Eigen::Vector3d(state[3], state[4], state[5]);
    return result;
}

// ============================================================================
// 内部辅助函数 (带参数版本)
// ============================================================================

namespace {

TrajectoryResult integrate_with_params(
    const TrajectoryState& initial_state,
    double target_distance,
    const Rk4Params& p,
    const Rk4TrajectorySolver& solver)
{
    TrajectoryResult result;
    result.hit = false;

    TrajectoryState state = initial_state;
    double t = 0;

    while (t < p.max_fly_time) {
        double horiz_dist = std::hypot(state[0], state[1]);

        if (horiz_dist >= target_distance) {
            result.hit = true;
            result.fly_time = t;
            result.hit_point = Eigen::Vector3d(state[0], state[1], state[2]);
            result.hit_velocity = Eigen::Vector3d(state[3], state[4], state[5]);
            return result;
        }

        if (state[2] < -10.0) {
            break;
        }

        state = solver.rk4_step(state, p.dt, p.drag);
        t += p.dt;
    }

    result.fly_time = t;
    result.hit_point = Eigen::Vector3d(state[0], state[1], state[2]);
    result.hit_velocity = Eigen::Vector3d(state[3], state[4], state[5]);
    return result;
}

double compute_height_error_with_params(
    double pitch, double yaw,
    double bullet_speed,
    const Eigen::Vector3d& vehicle_velocity,
    const Eigen::Vector3d& target_pos,
    const Rk4Params& p,
    const Rk4TrajectorySolver& solver)
{
    double cos_p = std::cos(pitch);
    double sin_p = std::sin(pitch);
    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);

    double vx0 = bullet_speed * cos_p * cos_y + vehicle_velocity.x();
    double vy0 = bullet_speed * cos_p * sin_y + vehicle_velocity.y();
    double vz0 = bullet_speed * sin_p + vehicle_velocity.z();

    TrajectoryState initial = {0, 0, 0, vx0, vy0, vz0};
    double target_horiz = std::hypot(target_pos.x(), target_pos.y());

    auto result = integrate_with_params(initial, target_horiz, p, solver);

    if (!result.hit) {
        return 1000.0;
    }

    return result.hit_point.z() - target_pos.z();
}

}  // namespace

// ============================================================================
// 求解接口
// ============================================================================

double Rk4TrajectorySolver::compute_height_error(
    double pitch,
    double yaw,
    double bullet_speed,
    const Eigen::Vector3d& vehicle_velocity,
    const Eigen::Vector3d& target_pos,
    const DragParams& params) const
{
    double cos_p = std::cos(pitch);
    double sin_p = std::sin(pitch);
    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);

    double vx0 = bullet_speed * cos_p * cos_y + vehicle_velocity.x();
    double vy0 = bullet_speed * cos_p * sin_y + vehicle_velocity.y();
    double vz0 = bullet_speed * sin_p + vehicle_velocity.z();

    TrajectoryState initial = {0, 0, 0, vx0, vy0, vz0};
    double target_horiz = std::hypot(target_pos.x(), target_pos.y());

    auto result = integrate(initial, target_horiz, params);

    if (!result.hit) {
        return 1000.0;
    }

    return result.hit_point.z() - target_pos.z();
}

AimResult Rk4TrajectorySolver::solve(const TrajectoryInput& input) const {
    return solve(input.target_pos, input.bullet_speed, input.vehicle_velocity);
}

AimResult Rk4TrajectorySolver::solve(
    const Eigen::Vector3d& target_pos,
    double bullet_speed,
    const Eigen::Vector3d& vehicle_velocity) const
{
    AimResult result;
    result.valid = false;

    // 参数检查
    double horiz_dist = std::hypot(target_pos.x(), target_pos.y());
    if (horiz_dist < 0.1 || bullet_speed < 5.0) {
        return result;
    }

    // 从配置读取所有参数
    Rk4Params p = Rk4Params::from_config();

    // 计算 yaw (水平方向直接指向)
    double yaw = std::atan2(target_pos.y(), target_pos.x());
    result.yaw = yaw;
    result.distance = target_pos.norm();

    // 二分法求 pitch
    double pitch_low = -M_PI / 4;   // -45°
    double pitch_high = M_PI / 3;   // 60° (吊射可能需要大仰角)

    // 初始猜测
    double pitch_guess = std::atan2(target_pos.z(), horiz_dist);
    pitch_guess = std::clamp(pitch_guess, pitch_low, pitch_high);

    // 检查边界
    double err_low = compute_height_error_with_params(
        pitch_low, yaw, bullet_speed, vehicle_velocity, target_pos, p, *this);
    double err_high = compute_height_error_with_params(
        pitch_high, yaw, bullet_speed, vehicle_velocity, target_pos, p, *this);

    // 确保 pitch_low 打低, pitch_high 打高
    if (err_low > 0 || err_high < 0) {
        // 尝试扩大范围
        pitch_low = -M_PI / 3;
        pitch_high = M_PI / 2.5;
        err_low = compute_height_error_with_params(
            pitch_low, yaw, bullet_speed, vehicle_velocity, target_pos, p, *this);
        err_high = compute_height_error_with_params(
            pitch_high, yaw, bullet_speed, vehicle_velocity, target_pos, p, *this);

        if (err_low > 0 || err_high < 0) {
            // 不可达
            result.pitch = pitch_guess;
            result.valid = false;
            return result;
        }
    }

    // 二分法迭代
    double pitch = pitch_guess;
    for (int iter = 0; iter < p.max_iter; ++iter) {
        double err = compute_height_error_with_params(
            pitch, yaw, bullet_speed, vehicle_velocity, target_pos, p, *this);

        if (std::abs(err) < p.tolerance) {
            break;
        }

        if (err > 0) {
            pitch_high = pitch;
        } else {
            pitch_low = pitch;
        }

        pitch = (pitch_low + pitch_high) / 2.0;
    }

    result.pitch = pitch;
    result.valid = true;

    // 计算飞行时间
    double cos_p = std::cos(pitch);
    double sin_p = std::sin(pitch);
    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);

    double vx0 = bullet_speed * cos_p * cos_y + vehicle_velocity.x();
    double vy0 = bullet_speed * cos_p * sin_y + vehicle_velocity.y();
    double vz0 = bullet_speed * sin_p + vehicle_velocity.z();

    TrajectoryState initial = {0, 0, 0, vx0, vy0, vz0};
    auto traj_result = integrate_with_params(initial, horiz_dist, p, *this);

    result.fly_time = traj_result.fly_time;
    result.hit_point = traj_result.hit_point;

    return result;
}

}  // namespace autoaim::fire_control
