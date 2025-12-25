/**
 * @file gimbal_planner.cpp
 * @brief 云台 MPC 轨迹规划器实现
 */

#include "gimbal_planner.hpp"

#include <cmath>
#include <algorithm>

#include "tinympc/tiny_api.hpp"

namespace autoaim::fire_control {

GimbalPlanner::GimbalPlanner(const Config& config)
    : config_(config)
{
    // 创建弹道解算器
    TrajectorySolver::Config traj_config;
    trajectory_solver_ = std::make_unique<TrajectorySolver>(traj_config);

    init_solvers();
}

GimbalPlanner::~GimbalPlanner()
{
    if (yaw_solver_) {
        tinympc::tiny_cleanup(yaw_solver_);
    }
    if (pitch_solver_) {
        tinympc::tiny_cleanup(pitch_solver_);
    }
}

void GimbalPlanner::set_config(const Config& config)
{
    config_ = config;

    // 重新初始化求解器
    if (yaw_solver_) {
        tinympc::tiny_cleanup(yaw_solver_);
        yaw_solver_ = nullptr;
    }
    if (pitch_solver_) {
        tinympc::tiny_cleanup(pitch_solver_);
        pitch_solver_ = nullptr;
    }
    initialized_ = false;
    init_solvers();
}

void GimbalPlanner::init_solvers()
{
    if (initialized_) return;

    const int nx = 2;  // [position, velocity]
    const int nu = 1;  // [acceleration]
    const int N = config_.horizon;
    const double dt = config_.dt;

    // 双积分器模型
    // x[k+1] = A * x[k] + B * u[k]
    // A = [1, dt]    B = [0 ]
    //     [0,  1]        [dt]
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
        Q_diag << config_.q_yaw_pos, config_.q_yaw_vel;

        Eigen::Matrix<double, 1, 1> R_diag;
        R_diag << config_.r_yaw_acc;

        tinympc::tiny_setup(
            &yaw_solver_,
            A, B, f,
            Q_diag.asDiagonal(),
            R_diag.asDiagonal(),
            config_.rho, nx, nu, N, 0
        );

        // 设置约束
        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(nx, N, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(nx, N, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(nu, N - 1, -config_.max_yaw_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(nu, N - 1, config_.max_yaw_acc);

        tinympc::tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);
        yaw_solver_->settings->max_iter = config_.max_iter;
    }

    // ==================== Pitch 求解器 ====================
    {
        Eigen::Matrix<double, 2, 1> Q_diag;
        Q_diag << config_.q_pitch_pos, config_.q_pitch_vel;

        Eigen::Matrix<double, 1, 1> R_diag;
        R_diag << config_.r_pitch_acc;

        tinympc::tiny_setup(
            &pitch_solver_,
            A, B, f,
            Q_diag.asDiagonal(),
            R_diag.asDiagonal(),
            config_.rho, nx, nu, N, 0
        );

        // 设置约束
        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(nx, N, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(nx, N, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(nu, N - 1, -config_.max_pitch_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(nu, N - 1, config_.max_pitch_acc);

        tinympc::tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);
        pitch_solver_->settings->max_iter = config_.max_iter;
    }

    initialized_ = true;
}

GimbalPlan GimbalPlanner::plan(
    const predictor::VehicleState& target,
    double current_yaw,
    double current_pitch,
    double current_yaw_vel,
    double current_pitch_vel,
    double bullet_speed
)
{
    GimbalPlan result;

    if (!target.valid) {
        return result;
    }

    // 生成参考轨迹 (同济方案：每步选最优装甲板)
    Trajectory ref = generate_reference(target, current_yaw, bullet_speed);

    // 提取 yaw 和 pitch 参考
    Eigen::Matrix<double, 2, Eigen::Dynamic> yaw_ref = ref.topRows(2);
    Eigen::Matrix<double, 2, Eigen::Dynamic> pitch_ref = ref.bottomRows(2);

    // 求解 yaw
    result.yaw = solve_axis(yaw_solver_, yaw_ref, 0, current_yaw_vel);

    // 求解 pitch
    result.pitch = solve_axis(pitch_solver_, pitch_ref, current_pitch, current_pitch_vel);

    // 恢复绝对 yaw
    result.yaw.position = normalize_angle(result.yaw.position + current_yaw);

    // 目标位置 (不含补偿)
    result.target_yaw = normalize_angle(yaw_ref(0, config_.half_horizon) + current_yaw);
    result.target_pitch = pitch_ref(0, config_.half_horizon);

    // ==================== 稳态偏差补偿 ====================
    // 电控 PID 跟踪斜坡函数时存在稳态偏差 = yaw_v * t0
    // 需要发送 yaw + t0 * yaw_v 来补偿
    // 这样在稳态时，电控实际达到的角度 = 发送角度 - t0 * yaw_v = 期望角度
    //
    // 参考: rm.cv.fans 延迟模型
    double t0 = config_.steady_state_t0;
    result.yaw.position = normalize_angle(result.yaw.position + t0 * result.yaw.velocity);
    result.pitch.position = result.pitch.position + t0 * result.pitch.velocity;

    // 计算当前跟踪误差 (用于调试)
    double yaw_err = normalize_angle(result.yaw.position - result.target_yaw - t0 * result.yaw.velocity);
    double pitch_err = result.pitch.position - result.target_pitch - t0 * result.pitch.velocity;
    result.tracking_error = std::hypot(yaw_err, pitch_err);

    // 计算开火误差 (考虑稳态后的实际误差)
    compute_fire_decision(result, yaw_ref, pitch_ref);

    result.valid = true;
    return result;
}

GimbalPlan GimbalPlanner::plan_simple(
    const Eigen::Vector3d& target_pos,
    double current_yaw,
    double current_pitch,
    double current_yaw_vel,
    double current_pitch_vel,
    double bullet_speed
)
{
    GimbalPlan result;

    // 弹道解算
    AimResult aim = trajectory_solver_->solve(target_pos, bullet_speed);
    if (!aim.valid) {
        return result;
    }

    // 构造简单参考轨迹 (恒定目标)
    const int N = config_.horizon;
    Eigen::Matrix<double, 2, Eigen::Dynamic> yaw_ref(2, N);
    Eigen::Matrix<double, 2, Eigen::Dynamic> pitch_ref(2, N);

    double target_yaw_rel = normalize_angle(aim.yaw - current_yaw);

    for (int i = 0; i < N; ++i) {
        yaw_ref(0, i) = target_yaw_rel;
        yaw_ref(1, i) = 0;  // 目标静止

        pitch_ref(0, i) = aim.pitch;
        pitch_ref(1, i) = 0;
    }

    // 求解
    result.yaw = solve_axis(yaw_solver_, yaw_ref, 0, current_yaw_vel);
    result.pitch = solve_axis(pitch_solver_, pitch_ref, current_pitch, current_pitch_vel);

    // 恢复绝对 yaw
    result.yaw.position = normalize_angle(result.yaw.position + current_yaw);

    result.target_yaw = aim.yaw;
    result.target_pitch = aim.pitch;

    double yaw_err = normalize_angle(result.yaw.position - result.target_yaw);
    double pitch_err = result.pitch.position - result.target_pitch;
    result.tracking_error = std::hypot(yaw_err, pitch_err);

    result.valid = true;
    return result;
}

GimbalPlanner::Trajectory GimbalPlanner::generate_reference(
    const predictor::VehicleState& target,
    double yaw0,
    double bullet_speed
)
{
    const int N = config_.horizon;
    const double dt = config_.dt;
    const int half = config_.half_horizon;

    Trajectory traj(4, N);

    // 判断是否高速陀螺
    bool is_high_spin = target.spin.active &&
                        target.spin.level == predictor::SpinLevel::HIGH;

    // 参考轨迹时间基准:
    // - MPC 的 half_horizon 对应当前 predict 时刻
    // - 需要预测到 control 时刻云台应该指向的位置
    // - control 时刻发出的子弹需要 fly_time 后击中
    //
    // 时间轴:
    //   img ──→ predict (现在) ──→ control ──→ hit
    //    │                            │          │
    //    │<── img_to_control ────────>│          │
    //    │                            │<────────>│
    //    │                              fly_time
    //
    // 预测时间 = (i - half) * dt + img_to_control + fly_time

    for (int i = 0; i < N; ++i) {
        // 相对于 predict 时刻的时间偏移
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
        AimResult rough_aim = trajectory_solver_->solve(rough_pos, bullet_speed);
        double fly_time = rough_aim.valid ? rough_aim.fly_time : 0.1;

        // 完整预测时间 = img_to_control + fly_time
        // 但 target 的状态是基于 img 时刻的，所以要加上 t_from_predict
        double total_predict_time = t_from_predict + config_.img_to_control + fly_time;

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
        AimResult aim = trajectory_solver_->solve(pos, bullet_speed);
        if (!aim.valid) {
            if (i > 0) {
                traj.col(i) = traj.col(i - 1);
            } else {
                traj.col(i).setZero();
            }
            continue;
        }

        // 相对 yaw
        double yaw_rel = normalize_angle(aim.yaw - yaw0);

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
        // 预测装甲板位置
        Eigen::Vector3d pos = target.predict_armor_position(i, t);

        // 计算装甲板朝向角 (需要从 armor state 获取或计算)
        // 简化：使用水平距离最近的装甲板 (同济做法)
        double horiz_dist = pos.head<2>().norm();

        // 过滤：太远的不考虑
        if (horiz_dist > 10.0) continue;

        // 过滤：角度太大的不考虑 (侧面装甲板)
        // 注意：这里需要预测 t 时刻的 z_to_v，简化处理用距离
        // 更精确的做法是计算装甲板法向量与视线的夹角

        if (horiz_dist < min_dist) {
            min_dist = horiz_dist;
            best_idx = i;
            out_pos = pos;
        }
    }

    return best_idx;
}

void GimbalPlanner::compute_fire_decision(
    GimbalPlan& plan,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& yaw_ref,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& pitch_ref
) const
{
    // 开火决策: MPC 规划轨迹与参考轨迹在 half_horizon 处的误差
    //
    // 根据 rm.cv.fans 延迟模型:
    // - control_to_fire 不参与预测时间计算
    // - 假设子弹像水流一样持续发射
    // - 只要 control 时刻瞄准正确，那一刻发出的子弹就能命中
    //
    // 因此开火决策只看当前时刻（half_horizon）的误差

    int fire_idx = config_.half_horizon;

    // 获取 MPC 规划的轨迹在 fire_idx 时刻的位置
    double planned_yaw = yaw_solver_->work->x(0, fire_idx);
    double planned_pitch = pitch_solver_->work->x(0, fire_idx);

    // 获取参考轨迹在 fire_idx 时刻的位置
    double ref_yaw = yaw_ref(0, fire_idx);
    double ref_pitch = pitch_ref(0, fire_idx);

    // 计算误差
    double yaw_err = normalize_angle(planned_yaw - ref_yaw);
    double pitch_err = planned_pitch - ref_pitch;

    plan.fire_error = std::hypot(yaw_err, pitch_err);
    plan.can_fire = plan.fire_error < config_.fire_threshold;
}

AxisPlan GimbalPlanner::solve_axis(
    tinympc::TinySolver* solver,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref,
    double pos0,
    double vel0
)
{
    AxisPlan result;

    if (!solver) return result;

    // 设置初始状态
    Eigen::VectorXd x0(2);
    x0 << pos0, vel0;
    tinympc::tiny_set_x0(solver, x0);

    // 设置参考轨迹
    solver->work->Xref = ref;

    // 求解
    tinympc::tiny_solve(solver);

    // 提取结果
    const int half = config_.half_horizon;
    result.position = solver->work->x(0, half);
    result.velocity = solver->work->x(1, half);
    result.acceleration = solver->work->u(0, half);

    return result;
}

double GimbalPlanner::normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

}  // namespace autoaim::fire_control
