/**
 * @file outpost_motion.cpp
 * @brief 前哨站运动模型实现 (EKF)
 */

#include "outpost_motion.hpp"

#include <cmath>

#include "aimer/common/math/math.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

// ============================================================================
// EKF 参数
// ============================================================================

namespace {

double get_double_param(const std::string& name, double default_val) {
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (auto* val = std::get_if<double>(&*ptr)) {
            return *val;
        }
    }
    return default_val;
}

// 过程噪声
double get_q_pos() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.q_pos", 0.1);
}

double get_q_vel() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.q_vel", 0.5);
}

double get_q_theta() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.q_theta", 0.05);
}

double get_q_omega() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.q_omega", 0.1);
}

// 观测噪声
double get_r_angle() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.r_angle", 0.01);
}

double get_r_dis_k() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.r_dis_k", 0.05);
}

double get_r_armor_yaw() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.r_armor_yaw", 0.1);
}

// 角速度约束容差
double get_omega_tolerance() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.omega_tolerance", 0.3);
}

}  // namespace

// ============================================================================
// OutpostMotion 实现
// ============================================================================

OutpostMotion::OutpostMotion() {
    slot_dz_.fill(0);
}

void OutpostMotion::init(const ArmorData& armor, double timestamp) {
    const auto& obs = armor.observation;

    double xa = obs.pos.x();
    double ya = obs.pos.y();
    double za = obs.pos.z();
    double armor_yaw = obs.z[obs::ARMOR_YAW];

    // 从装甲板位置反推旋转中心
    double xc = xa + outpost::RADIUS * std::cos(armor_yaw);
    double yc = ya + outpost::RADIUS * std::sin(armor_yaw);
    double zc = za;  // 初始假设 dz = 0

    // 初始化状态
    VectorX x0 = VectorX::Zero();
    x0[outpost::XC] = xc;
    x0[outpost::YC] = yc;
    x0[outpost::ZC] = zc;
    x0[outpost::THETA] = armor_yaw;
    x0[outpost::OMEGA] = 0;  // 初始为0，让 EKF 自己判断方向

    ekf_.init(x0);

    // 初始化槽位高度差
    slot_dz_.fill(0);           // 先全部清零
    slot_dz_[0] = 0;            // 第一个装甲板定为槽位0，高度差为0
    slot_known_.fill(false);
    slot_known_[0] = true;      // 槽位0已知

    current_slot_ = 0;
    current_dz_ = 0;

    // 快速收敛：方向待判断
    omega_sign_determined_ = false;

    tracking_armor_id_ = armor.id;
    last_update_time_ = timestamp;
    initialized_ = true;
}

void OutpostMotion::update(const ArmorData& armor, double timestamp) {
    if (!initialized_) {
        init(armor, timestamp);
        return;
    }

    const auto& obs = armor.observation;

    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    // 处理装甲板切换
    handle_armor_switch(armor);

    // 预测
    OutpostCVPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward(predict_func, Q);

    // 观测
    double armor_yaw = obs.z[obs::ARMOR_YAW];

    // 连续化 yaw
    VectorX x = ekf_.get_x();
    double theta_pred = x[outpost::THETA];
    double yaw_continuous = theta_pred + math::angle_diff(theta_pred, armor_yaw);

    // 构建观测向量
    VectorZ z;
    z[outpost::YAW] = obs.z[obs::YAW];
    z[outpost::PITCH] = obs.z[obs::PITCH];
    z[outpost::DIS] = obs.z[obs::DIST];
    z[outpost::ARMOR_YAW] = yaw_continuous;

    // 观测更新
    OutpostMeasure measure_func(current_dz_);
    MatrixZZ R = build_R(obs.z[obs::DIST], armor.z_to_v());
    ekf_.update_forward(measure_func, z, R);

    // 约束角速度
    constrain_omega();

    // 更新当前槽位的高度差 (持续学习)
    x = ekf_.get_x();
    double new_dz = obs.pos.z() - x[outpost::ZC];
    slot_dz_[current_slot_] = new_dz;
    slot_known_[current_slot_] = true;
    current_dz_ = new_dz;

    last_update_time_ = timestamp;
}

bool OutpostMotion::handle_armor_switch(const ArmorData& armor) {
    if (armor.id == tracking_armor_id_ || tracking_armor_id_ < 0) {
        tracking_armor_id_ = armor.id;
        return false;
    }

    // 装甲板切换了
    const auto& obs = armor.observation;
    VectorX x = ekf_.get_x();
    double theta_pred = x[outpost::THETA];
    double armor_yaw = obs.z[obs::ARMOR_YAW];

    // 计算角度差，判断槽位变化方向
    double theta_diff = math::angle_diff(theta_pred, armor_yaw);

    // 更新相位
    x[outpost::THETA] = theta_pred + theta_diff;

    // 通过角度差判断槽位变化 (相隔约120度)
    int new_slot = current_slot_;
    int direction = 0;  // +1: 下一个槽位, -1: 上一个槽位
    if (theta_diff > M_PI / 3) {
        new_slot = (current_slot_ + 1) % 3;
        direction = 1;
    } else if (theta_diff < -M_PI / 3) {
        new_slot = (current_slot_ + 2) % 3;
        direction = -1;
    }

    // 记录新槽位的高度差
    double new_dz = obs.pos.z() - x[outpost::ZC];
    slot_dz_[new_slot] = new_dz;
    slot_known_[new_slot] = true;

    // 利用 Δz = 10cm 约束，推算第三个槽位的高度差
    if (direction != 0) {
        double delta_z = new_dz - current_dz_;  // 本次切换的高度变化
        int third_slot = (new_slot + direction + 3) % 3;

        if (!slot_known_[third_slot]) {
            // 第三个槽位未知，推算它
            // 如果 delta_z > 0，说明顺着这个方向是递增的
            slot_dz_[third_slot] = new_dz + delta_z;
            slot_known_[third_slot] = true;
        }
    }

    current_slot_ = new_slot;
    current_dz_ = new_dz;

    ekf_.set_x(x);
    tracking_armor_id_ = armor.id;
    return true;
}

void OutpostMotion::constrain_omega() {
    VectorX x = ekf_.get_x();
    double omega = x[outpost::OMEGA];

    if (!omega_sign_determined_) {
        // 还没确定方向，等 EKF 估计超过阈值
        // 阈值设为 0.6π (75% of 0.8π)，更安全
        constexpr double OMEGA_THRESHOLD = 0.6 * M_PI;
        if (omega > OMEGA_THRESHOLD) {
            x[outpost::OMEGA] = outpost::OMEGA_ABS;  // +0.8π 逆时针
            omega_sign_determined_ = true;
            ekf_.set_x(x);
        } else if (omega < -OMEGA_THRESHOLD) {
            x[outpost::OMEGA] = -outpost::OMEGA_ABS;  // -0.8π 顺时针
            omega_sign_determined_ = true;
            ekf_.set_x(x);
        }
        // 未达阈值则不修改，让 EKF 继续估计
    } else {
        // 已确定方向，只约束绝对值
        double tolerance = get_omega_tolerance();
        if (std::abs(std::abs(omega) - outpost::OMEGA_ABS) > tolerance) {
            x[outpost::OMEGA] = std::copysign(outpost::OMEGA_ABS, omega);
            ekf_.set_x(x);
        }
    }
}

OutpostMotion::MatrixXX OutpostMotion::build_Q(double dt) const {
    MatrixXX Q = MatrixXX::Zero();

    double q_pos = get_q_pos();
    double q_vel = get_q_vel();
    double q_theta = get_q_theta();
    double q_omega = get_q_omega();

    // 前哨站 ω 是规则固定的，方向确定后过程噪声应该很小
    if (omega_sign_determined_) {
        q_omega *= 0.01;  // 方向确定后，ω 几乎不变
    }

    Q(outpost::XC, outpost::XC) = q_pos * dt;
    Q(outpost::VX, outpost::VX) = q_vel * dt;
    Q(outpost::YC, outpost::YC) = q_pos * dt;
    Q(outpost::VY, outpost::VY) = q_vel * dt;
    Q(outpost::ZC, outpost::ZC) = q_pos * dt * 0.1;  // Z 变化很小
    Q(outpost::THETA, outpost::THETA) = q_theta * dt;
    Q(outpost::OMEGA, outpost::OMEGA) = q_omega * dt;

    return Q;
}

OutpostMotion::MatrixZZ OutpostMotion::build_R(double distance, double z_to_v) const {
    MatrixZZ R = MatrixZZ::Zero();

    double r_angle = get_r_angle();
    double r_dis_k = get_r_dis_k();
    double r_armor_yaw = get_r_armor_yaw();

    // 侧面观看时距离噪声更大 (参考 sp_vision_25)
    // z_to_v 越大越侧面，log(|z_to_v| + 1) + 1 作为缩放因子
    double side_factor = std::log(std::abs(z_to_v) + 1) + 1;

    // 距离越远角度噪声越大
    double dist_factor = std::log(distance + 1) / 200 + 1;

    R(outpost::YAW, outpost::YAW) = r_angle * dist_factor;
    R(outpost::PITCH, outpost::PITCH) = r_angle * dist_factor;
    R(outpost::DIS, outpost::DIS) = r_dis_k * distance * distance * side_factor;
    R(outpost::ARMOR_YAW, outpost::ARMOR_YAW) = r_armor_yaw;

    return R;
}

Eigen::Vector3d OutpostMotion::predict_center(double dt) const {
    VectorX x = ekf_.get_x();
    return Eigen::Vector3d(
        x[outpost::XC] + x[outpost::VX] * dt,
        x[outpost::YC] + x[outpost::VY] * dt,
        x[outpost::ZC]
    );
}

Eigen::Vector3d OutpostMotion::predict_armor_pos(int slot, double dt) const {
    VectorX x = ekf_.get_x();

    double xc = x[outpost::XC] + x[outpost::VX] * dt;
    double yc = x[outpost::YC] + x[outpost::VY] * dt;
    double zc = x[outpost::ZC];

    double theta = x[outpost::THETA] + x[outpost::OMEGA] * dt;

    // 槽位角度偏移 (相隔120度)
    double angle_offset = slot * (2.0 * M_PI / 3);
    double armor_theta = theta + angle_offset;

    // 查表获取高度差 (已学习的槽位)
    double dz = slot_dz_[slot];

    double xa = xc - outpost::RADIUS * std::cos(armor_theta);
    double ya = yc - outpost::RADIUS * std::sin(armor_theta);
    double za = zc + dz;

    return Eigen::Vector3d(xa, ya, za);
}

Eigen::Vector3d OutpostMotion::get_armor_pos() const {
    return predict_armor_pos(current_slot_, 0);
}

Eigen::Vector3d OutpostMotion::get_velocity() const {
    VectorX x = ekf_.get_x();
    return Eigen::Vector3d(x[outpost::VX], x[outpost::VY], 0);
}

void OutpostMotion::reset() {
    initialized_ = false;
    slot_dz_.fill(0);
    slot_known_.fill(false);
    tracking_armor_id_ = -1;
    current_slot_ = 0;
    current_dz_ = 0;
}

}  // namespace autoaim::predictor
