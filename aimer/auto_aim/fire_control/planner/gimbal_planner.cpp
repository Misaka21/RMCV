/**
 * @file gimbal_planner.cpp
 * @brief 云台 MPC 规划器实现
 */

#include "gimbal_planner.hpp"

#include <cmath>

#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

// ==================== 简易弹道 ====================

Eigen::Vector2d GimbalPlanner::simple_aim(
    const Eigen::Vector3d& armor_pos_world,
    double bullet_speed,
    const Eigen::Quaterniond& q_imu)
{
    // 转到枪管坐标系
    const Eigen::Vector3d barrel_pos =
        aimer::tf::world_to_barrel_origin_world(armor_pos_world, q_imu);

    if (barrel_pos.norm() < 1e-3) {
        return {0.0, 0.0};
    }

    // yaw: atan2(Y, X) — 枪管 X 前 Y 左
    double yaw = std::atan2(barrel_pos.y(), barrel_pos.x());

    // pitch: 无阻力抛物线
    constexpr double g = 9.7833;
    const double d = std::hypot(barrel_pos.x(), barrel_pos.y());
    const double h = barrel_pos.z();

    if (bullet_speed < 10.0) {
        return {yaw, std::atan2(h, d)};
    }

    const double v2 = bullet_speed * bullet_speed;
    const double a = g * d * d / (2.0 * v2);
    const double b = -d;
    const double c = a + h;
    const double delta = b * b - 4.0 * a * c;

    double pitch = std::atan2(h, d);  // fallback: 几何俯仰
    if (delta >= 0.0 && std::abs(a) > 1e-12) {
        const double sqrt_delta = std::sqrt(delta);
        const double tan1 = (-b + sqrt_delta) / (2.0 * a);
        const double tan2 = (-b - sqrt_delta) / (2.0 * a);
        const double pitch1 = std::atan(tan1);
        const double pitch2 = std::atan(tan2);
        // 选低伸弹道 (较短的飞行时间)
        // RMCV2026 pitch 正=上, 直接用解出的角度 (OSS 用负号是因为其坐标系不同)
        pitch = (std::abs(pitch1) < std::abs(pitch2)) ? pitch1 : pitch2;
    }

    return {yaw, pitch};
}

// ==================== 构造 ====================

GimbalPlanner::GimbalPlanner(const PlannerConfig& cfg)
    : cfg_(cfg)
    , yaw_mpc_(cfg.dt, cfg.horizon, cfg.q_pos, cfg.q_vel, cfg.r, cfg.rho)
    , pitch_mpc_(cfg.dt, cfg.horizon, cfg.q_pos, cfg.q_vel, cfg.r, cfg.rho)
{
    // 设置加速度约束
    const int N = cfg.horizon;
    Eigen::Matrix<double, 1, Eigen::Dynamic> yaw_lb(1, N - 1);
    Eigen::Matrix<double, 1, Eigen::Dynamic> yaw_ub(1, N - 1);
    Eigen::Matrix<double, 1, Eigen::Dynamic> pitch_lb(1, N - 1);
    Eigen::Matrix<double, 1, Eigen::Dynamic> pitch_ub(1, N - 1);

    yaw_lb.setConstant(-cfg.max_yaw_acc);
    yaw_ub.setConstant(cfg.max_yaw_acc);
    pitch_lb.setConstant(-cfg.max_pitch_acc);
    pitch_ub.setConstant(cfg.max_pitch_acc);

    yaw_mpc_.set_input_bounds(yaw_lb, yaw_ub);
    pitch_mpc_.set_input_bounds(pitch_lb, pitch_ub);

    yaw_ref_ = Eigen::Matrix2Xd::Zero(2, N);
    pitch_ref_ = Eigen::Matrix2Xd::Zero(2, N);
}

// ==================== 参考轨迹生成 ====================

void GimbalPlanner::build_reference(
    const predictor::TargetState& target,
    int armor_idx,
    double img_age,
    double hit_offset,
    double bullet_speed,
    const Eigen::Vector3d& /*self_velocity*/,
    const Eigen::Quaterniond& q_imu)
{
    const int N = cfg_.horizon;
    const double DT = cfg_.dt;

    // 调整范围
    const double bs = std::clamp(bullet_speed, 10.0, 25.0);

    // 计算各时刻的瞄准角
    for (int k = 0; k < N; ++k) {
        const double t = k * DT;
        const double dt_from_snap = img_age + t + hit_offset;

        // 预测装甲板位置
        const Eigen::Vector3d armor_pos =
            target.predict_armor_position(armor_idx, dt_from_snap);

        if (!armor_pos.allFinite()) {
            // 退化: 沿用上一时刻
            if (k > 0) {
                yaw_ref_.col(k) = yaw_ref_.col(k - 1);
                pitch_ref_.col(k) = pitch_ref_.col(k - 1);
            }
            continue;
        }

        const Eigen::Vector2d yp = simple_aim(armor_pos, bs, q_imu);
        yaw_ref_(0, k) = yp.x();
        pitch_ref_(0, k) = yp.y();
    }

    // yaw 解缠绕: 消除 ±π 跳变, 使轨迹连续
    double cum_offset = 0.0;
    double prev_yaw = yaw_ref_(0, 0);
    for (int k = 1; k < N; ++k) {
        double diff = yaw_ref_(0, k) - prev_yaw;
        // 将 diff 归一到 [-π, π]
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        cum_offset += diff;
        yaw_ref_(0, k) = yaw_ref_(0, 0) + cum_offset;
        prev_yaw = yaw_ref_(0, k);
    }
    // 保存展开的中心 yaw (用于输出时归一化)
    yaw_unwrap_offset_ = yaw_ref_(0, 0);

    // 中心差分计算角速度
    for (int k = 1; k < N - 1; ++k) {
        yaw_ref_(1, k) = (yaw_ref_(0, k + 1) - yaw_ref_(0, k - 1)) / (2.0 * DT);
        pitch_ref_(1, k) = (pitch_ref_(0, k + 1) - pitch_ref_(0, k - 1)) / (2.0 * DT);
    }
    // 边界: 前向/后向差分
    if (N >= 2) {
        yaw_ref_(1, 0) = (yaw_ref_(0, 1) - yaw_ref_(0, 0)) / DT;
        pitch_ref_(1, 0) = (pitch_ref_(0, 1) - pitch_ref_(0, 0)) / DT;
        yaw_ref_(1, N - 1) = (yaw_ref_(0, N - 1) - yaw_ref_(0, N - 2)) / DT;
        pitch_ref_(1, N - 1) = (pitch_ref_(0, N - 1) - pitch_ref_(0, N - 2)) / DT;
    }

    has_reference_ = true;
}

// ==================== 实时求解 ====================

PlannerOutput GimbalPlanner::step(const GimbalState& gimbal) {
    PlannerOutput out;

    if (!has_reference_) {
        return out;
    }

    // 设置参考轨迹
    yaw_mpc_.set_reference(yaw_ref_);
    pitch_mpc_.set_reference(pitch_ref_);

    // 设置初始状态: 对齐当前云台实际位置+速度
    // yaw 需要展开以匹配参考轨迹的连续表示
    double yaw0 = aimer::math::reduced_angle(gimbal.yaw - yaw_unwrap_offset_)
        + yaw_unwrap_offset_;
    yaw_mpc_.set_initial_state(Eigen::Vector2d(yaw0, gimbal.yaw_vel));
    pitch_mpc_.set_initial_state(Eigen::Vector2d(gimbal.pitch, gimbal.pitch_vel));

    // ADMM 迭代 (warm-start 从上一次解)
    int max_iter = std::max(1, cfg_.max_iter);
    yaw_mpc_.solve(max_iter);
    pitch_mpc_.solve(max_iter);

    // 提取指定步的输出
    const int s = std::clamp(cfg_.cmd_step, 0, cfg_.horizon - 2);
    {
        const Eigen::Vector2d ys = yaw_mpc_.state_at(s);
        // yaw 归一化回 [-π, π]
        out.yaw = aimer::math::reduced_angle(ys.x());
        out.yaw_vel = ys.y();
        out.yaw_acc = yaw_mpc_.control_at(s);
    }
    {
        const Eigen::Vector2d ps = pitch_mpc_.state_at(s);
        out.pitch = ps.x();
        out.pitch_vel = ps.y();
        out.pitch_acc = pitch_mpc_.control_at(s);
    }

    out.valid = true;
    return out;
}

// ==================== 重置 ====================

void GimbalPlanner::reset() {
    has_reference_ = false;
    yaw_unwrap_offset_ = 0;
    yaw_ref_.setZero();
    pitch_ref_.setZero();
}

}  // namespace autoaim::fire_control
