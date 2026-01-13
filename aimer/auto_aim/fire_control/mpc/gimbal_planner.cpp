/**
 * @file gimbal_planner.cpp
 * @brief 云台 MPC 轨迹规划器实现
 */

#include "gimbal_planner.hpp"

#include <cmath>
#include <algorithm>
#include <tuple>

#include "tinympc/tiny_api.hpp"
#include "aimer/common/trajectory/solver_factory.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

GimbalPlanner::GimbalPlanner() = default;

GimbalPlanner::~GimbalPlanner()
{
    if (yaw_solver_) {
        tinympc::tiny_cleanup(yaw_solver_);
    }
    if (pitch_solver_) {
        tinympc::tiny_cleanup(pitch_solver_);
    }
}

void GimbalPlanner::reset()
{
    // TinyMPC 状态重置 (如果需要)
}

void GimbalPlanner::ensure_solvers_initialized()
{
    // 读取参数
    int horizon = static_cast<int>(runtime_param::get_param<int64_t>("AutoAim.FireControl.MPC.horizon"));
    double q_yaw_pos = runtime_param::get_param<double>("AutoAim.FireControl.MPC.q_yaw_pos");
    double q_pitch_pos = runtime_param::get_param<double>("AutoAim.FireControl.MPC.q_pitch_pos");

    // 检查参数是否变化，如果变化则重新初始化
    bool need_reinit = !initialized_ ||
                       horizon != cached_horizon_ ||
                       q_yaw_pos != cached_q_yaw_pos_ ||
                       q_pitch_pos != cached_q_pitch_pos_;

    if (!need_reinit) return;

    // 清理旧的 solver
    if (yaw_solver_) {
        tinympc::tiny_cleanup(yaw_solver_);
        yaw_solver_ = nullptr;
    }
    if (pitch_solver_) {
        tinympc::tiny_cleanup(pitch_solver_);
        pitch_solver_ = nullptr;
    }

    // 读取所有 MPC 参数
    double dt = runtime_param::get_param<double>("AutoAim.FireControl.MPC.dt");
    double q_yaw_vel = runtime_param::get_param<double>("AutoAim.FireControl.MPC.q_yaw_vel");
    double r_yaw_acc = runtime_param::get_param<double>("AutoAim.FireControl.MPC.r_yaw_acc");
    double max_yaw_acc = runtime_param::get_param<double>("AutoAim.FireControl.MPC.max_yaw_acc");
    double q_pitch_vel = runtime_param::get_param<double>("AutoAim.FireControl.MPC.q_pitch_vel");
    double r_pitch_acc = runtime_param::get_param<double>("AutoAim.FireControl.MPC.r_pitch_acc");
    double max_pitch_acc = runtime_param::get_param<double>("AutoAim.FireControl.MPC.max_pitch_acc");
    int max_iter = static_cast<int>(runtime_param::get_param<int64_t>("AutoAim.FireControl.MPC.max_iter"));
    double rho = runtime_param::get_param<double>("AutoAim.FireControl.MPC.rho");

    const int nx = 2;  // [position, velocity]
    const int nu = 1;  // [acceleration]
    const int N = horizon;

    // 双积分器模型
    Eigen::MatrixXd A(nx, nx);
    A << 1, dt,
         0, 1;

    Eigen::MatrixXd B(nx, nu);
    B << 0,
         dt;

    Eigen::VectorXd f = Eigen::VectorXd::Zero(nx);

    // ==================== Yaw 求解器 ====================
    {
        Eigen::Matrix<double, 2, 1> Q_diag;
        Q_diag << q_yaw_pos, q_yaw_vel;

        Eigen::Matrix<double, 1, 1> R_diag;
        R_diag << r_yaw_acc;

        tinympc::tiny_setup(
            &yaw_solver_,
            A, B, f,
            Q_diag.asDiagonal(),
            R_diag.asDiagonal(),
            rho, nx, nu, N, 0
        );

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(nx, N, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(nx, N, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(nu, N - 1, -max_yaw_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(nu, N - 1, max_yaw_acc);

        tinympc::tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);
        yaw_solver_->settings->max_iter = max_iter;
    }

    // ==================== Pitch 求解器 ====================
    {
        Eigen::Matrix<double, 2, 1> Q_diag;
        Q_diag << q_pitch_pos, q_pitch_vel;

        Eigen::Matrix<double, 1, 1> R_diag;
        R_diag << r_pitch_acc;

        tinympc::tiny_setup(
            &pitch_solver_,
            A, B, f,
            Q_diag.asDiagonal(),
            R_diag.asDiagonal(),
            rho, nx, nu, N, 0
        );

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(nx, N, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(nx, N, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(nu, N - 1, -max_pitch_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(nu, N - 1, max_pitch_acc);

        tinympc::tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);
        pitch_solver_->settings->max_iter = max_iter;
    }

    // 缓存参数
    cached_horizon_ = horizon;
    cached_q_yaw_pos_ = q_yaw_pos;
    cached_q_pitch_pos_ = q_pitch_pos;
    initialized_ = true;
}

GimbalPlan GimbalPlanner::plan(
    const predictor::VehicleState& target,
    const GimbalState& gimbal,
    const LatencyInfo& latency,
    double bullet_speed
)
{
    GimbalPlan result;

    if (!target.valid) {
        return result;
    }

    // 确保 solver 已初始化
    ensure_solvers_initialized();

    // 读取参数
    int half_horizon = static_cast<int>(runtime_param::get_param<int64_t>("AutoAim.FireControl.MPC.half_horizon"));

    // 生成参考轨迹
    Trajectory ref = generate_reference(target, gimbal, latency, bullet_speed);

    // 提取 yaw 和 pitch 参考
    Eigen::Matrix<double, 2, Eigen::Dynamic> yaw_ref = ref.topRows(2);
    Eigen::Matrix<double, 2, Eigen::Dynamic> pitch_ref = ref.bottomRows(2);

    // 求解 yaw (使用相对角度)
    auto [yaw_pos, yaw_vel, yaw_acc] = solve_axis(yaw_solver_, yaw_ref, 0, gimbal.yaw_vel);

    // 求解 pitch
    auto [pitch_pos, pitch_vel, pitch_acc] = solve_axis(pitch_solver_, pitch_ref, gimbal.pitch, gimbal.pitch_vel);

    // 恢复绝对 yaw
    result.yaw = normalize_angle(yaw_pos + gimbal.yaw);
    result.yaw_vel = yaw_vel;
    result.yaw_acc = yaw_acc;

    result.pitch = pitch_pos;
    result.pitch_vel = pitch_vel;
    result.pitch_acc = pitch_acc;

    result.valid = true;
    return result;
}

GimbalPlanner::Trajectory GimbalPlanner::generate_reference(
    const predictor::VehicleState& target,
    const GimbalState& gimbal,
    const LatencyInfo& latency,
    double bullet_speed
)
{
    // 读取参数
    int horizon = static_cast<int>(runtime_param::get_param<int64_t>("AutoAim.FireControl.MPC.horizon"));
    int half_horizon = static_cast<int>(runtime_param::get_param<int64_t>("AutoAim.FireControl.MPC.half_horizon"));
    double dt = runtime_param::get_param<double>("AutoAim.FireControl.MPC.dt");

    // 使用传入的 latency
    double img_to_control = latency.img_to_predict + latency.predict_to_send + latency.send_to_control;

    const int N = horizon;
    const int half = half_horizon;

    Trajectory traj(4, N);

    // 判断是否高速陀螺
    bool is_high_spin = target.spin.active &&
                        target.spin.level == predictor::SpinLevel::HIGH;

    for (int i = 0; i < N; ++i) {
        double t_from_predict = (i - half) * dt;

        // 首先用粗略位置估计飞行时间
        Eigen::Vector3d rough_pos;
        if (is_high_spin) {
            rough_pos = target.predict_center(t_from_predict);
        } else {
            int idx = select_best_armor_at_time(target, t_from_predict, rough_pos);
            if (idx < 0) {
                rough_pos = target.predict_center(t_from_predict);
            }
        }

        // 弹道解算获取飞行时间
        AimResult rough_aim = ::fire_control::trajectory::solve(rough_pos, bullet_speed);
        double fly_time = rough_aim.valid ? rough_aim.fly_time : 0.1;

        // 完整预测时间
        double total_predict_time = t_from_predict + img_to_control + fly_time;

        // 预测目标在 hit 时刻的位置
        Eigen::Vector3d pos;
        if (is_high_spin) {
            pos = target.predict_center(total_predict_time);
        } else {
            int armor_idx = select_best_armor_at_time(target, total_predict_time, pos);
            if (armor_idx < 0) {
                pos = target.predict_center(total_predict_time);
            }
        }

        // 弹道解算得到瞄准角度
        AimResult aim = ::fire_control::trajectory::solve(pos, bullet_speed);
        if (!aim.valid) {
            if (i > 0) {
                traj.col(i) = traj.col(i - 1);
            } else {
                traj.col(i).setZero();
            }
            continue;
        }

        // 相对 yaw
        double yaw_rel = normalize_angle(aim.yaw - gimbal.yaw);

        traj(0, i) = yaw_rel;
        traj(2, i) = aim.pitch;
    }

    // 计算速度 (中心差分)
    for (int i = 1; i < N - 1; ++i) {
        traj(1, i) = normalize_angle(traj(0, i + 1) - traj(0, i - 1)) / (2 * dt);
        traj(3, i) = (traj(2, i + 1) - traj(2, i - 1)) / (2 * dt);
    }

    // 边界处理
    traj(1, 0) = traj(1, 1);
    traj(1, N - 1) = traj(1, N - 2);
    traj(3, 0) = traj(3, 1);
    traj(3, N - 1) = traj(3, N - 2);

    return traj;
}

int GimbalPlanner::select_best_armor_at_time(
    const predictor::VehicleState& target,
    double t,
    Eigen::Vector3d& out_pos
) const
{
    int best_idx = -1;
    double min_dist = 1e10;

    for (int i = 0; i < target.armor_count; ++i) {
        Eigen::Vector3d pos = target.predict_armor_position(i, t);
        double horiz_dist = pos.head<2>().norm();

        if (horiz_dist > 10.0) continue;

        if (horiz_dist < min_dist) {
            min_dist = horiz_dist;
            best_idx = i;
            out_pos = pos;
        }
    }

    return best_idx;
}

std::tuple<double, double, double> GimbalPlanner::solve_axis(
    tinympc::TinySolver* solver,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref,
    double pos0,
    double vel0
)
{
    if (!solver) return {pos0, 0, 0};

    int half_horizon = static_cast<int>(runtime_param::get_param<int64_t>("AutoAim.FireControl.MPC.half_horizon"));

    // 设置初始状态
    Eigen::VectorXd x0(2);
    x0 << pos0, vel0;
    tinympc::tiny_set_x0(solver, x0);

    // 设置参考轨迹
    solver->work->Xref = ref;

    // 求解
    tinympc::tiny_solve(solver);

    // 提取结果
    const int half = half_horizon;
    double position = solver->work->x(0, half);
    double velocity = solver->work->x(1, half);
    double acceleration = solver->work->u(0, half);

    return {position, velocity, acceleration};
}

double GimbalPlanner::normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

}  // namespace autoaim::fire_control
