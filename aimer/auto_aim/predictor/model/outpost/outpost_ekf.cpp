/**
 * @file outpost_motion.cpp
 * @brief 前哨站运动模型实现 (EKF)
 */

#include "outpost_ekf.hpp"

#include <cmath>
#include <limits>

#include "aimer/common/math/math.hpp"
#include "plugin/debug/logger.hpp"
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

// 是否锁定角速度到规则值 (关闭则让 EKF 自己估计)
bool get_lock_omega() {
    auto ptr = runtime_param::find_param("AutoAim.Predictor.OutpostEKF.lock_omega");
    if (ptr != nullptr) {
        if (auto* val = std::get_if<bool>(&*ptr)) {
            return *val;
        }
    }
    return true;  // 默认开启
}

// 判断旋转方向的阈值
double get_omega_direction_threshold() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.omega_direction_threshold", 0.6 * M_PI);
}

// 是否强制速度清零 (前哨站定点旋转)
bool get_force_zero_velocity() {
    auto ptr = runtime_param::find_param("AutoAim.Predictor.OutpostEKF.force_zero_velocity");
    if (ptr != nullptr) {
        if (auto* val = std::get_if<bool>(&*ptr)) {
            return *val;
        }
    }
    return true;  // 默认开启
}

// 门限检查参数
double get_chi2_threshold() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.Gating.chi2_threshold", 13.28);
}

int get_max_reject() {
    auto ptr = runtime_param::find_param("AutoAim.Predictor.OutpostEKF.Gating.max_reject");
    if (ptr != nullptr) {
        if (auto* val = std::get_if<int64_t>(&*ptr)) {
            return static_cast<int>(*val);
        }
    }
    return 5;
}

double get_q_scale_increase() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.Gating.q_scale_increase", 1.5);
}

double get_q_scale_decay() {
    return get_double_param("AutoAim.Predictor.OutpostEKF.Gating.q_scale_decay", 0.9);
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
    double xc = xa + outpost_model::RADIUS * std::cos(armor_yaw);
    double yc = ya + outpost_model::RADIUS * std::sin(armor_yaw);
    double zc = za;  // 初始假设 dz = 0

    // 初始化状态
    VectorX x0 = VectorX::Zero();
    x0[outpost_model::XC] = xc;
    x0[outpost_model::YC] = yc;
    x0[outpost_model::ZC] = zc;
    x0[outpost_model::THETA] = armor_yaw;
    x0[outpost_model::OMEGA] = 0;  // 初始为0，让 EKF 自己判断方向

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

    // 预测 (使用自适应缩放的过程噪声)
    OutpostCVPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward_scaled(predict_func, Q);

    // 观测
    double armor_yaw = obs.z[obs::ARMOR_YAW];
    VectorX x = ekf_.get_x();
    const int observed_slot = match_observed_slot(armor_yaw);
    const double slot_offset = observed_slot * (2.0 * M_PI / 3.0);
    const double predicted_slot_yaw = x[outpost_model::THETA] + slot_offset;
    const double slot_yaw_continuous =
        predicted_slot_yaw + aimer::math::angle_diff(predicted_slot_yaw, armor_yaw);
    const double base_theta_continuous = slot_yaw_continuous - slot_offset;

    OutpostMeasure measure_func(slot_dz_[observed_slot], slot_offset);
    VectorZ inner_z;
    double x_arr[outpost_model::N_X], z_arr[outpost_model::N_Z];
    for (int i = 0; i < outpost_model::N_X; ++i) x_arr[i] = x[i];
    measure_func(x_arr, z_arr);
    for (int i = 0; i < outpost_model::N_Z; ++i) inner_z[i] = z_arr[i];

    // 构建观测向量
    VectorZ z;
    z[outpost_model::YAW] = aimer::math::get_closest_angle(obs.z[obs::YAW], inner_z[outpost_model::YAW]);
    z[outpost_model::PITCH] = obs.z[obs::PITCH];
    z[outpost_model::DIS] = obs.z[obs::DIST];
    z[outpost_model::ARMOR_YAW] = slot_yaw_continuous;

    // 构建重置状态
    double xa = obs.pos.x();
    double ya = obs.pos.y();
    double za = obs.pos.z();
    double xc = xa + outpost_model::RADIUS * std::cos(armor_yaw);
    double yc = ya + outpost_model::RADIUS * std::sin(armor_yaw);
    double zc = za;

    VectorX reset_state = VectorX::Zero();
    reset_state[outpost_model::XC] = xc;
    reset_state[outpost_model::YC] = yc;
    reset_state[outpost_model::ZC] = zc;
    reset_state[outpost_model::THETA] = base_theta_continuous;
    reset_state[outpost_model::OMEGA] = 0;

    // 带门限检查的观测更新
    MatrixZZ R = build_R(obs.z[obs::DIST], armor.z_to_v());
    auto status = ekf_.update_forward_gated(
        measure_func, z, R, reset_state,
        get_chi2_threshold(), get_max_reject(), get_q_scale_increase(), get_q_scale_decay()
    );

    // 处理更新结果
    if (status == aimer::filter::UpdateStatus::RESET) {
        // EKF 已重置，同时重置外部状态
        slot_dz_.fill(0);
        slot_known_.fill(false);
        slot_dz_[observed_slot] = 0;
        slot_known_[observed_slot] = true;
        current_slot_ = observed_slot;
        current_dz_ = 0;
        omega_sign_determined_ = false;  // 需要重新判断方向
        debug::print(debug::PrintMode::WARNING, "OutpostMotion",
            "EKF reset due to {} consecutive rejections", get_max_reject());
    } else if (status == aimer::filter::UpdateStatus::REJECTED) {
        debug::print(debug::PrintMode::DEBUG, "OutpostMotion",
            "Observation rejected, q_scale={:.2f}", ekf_.get_q_scale());
    }

    // 约束角速度
    constrain_omega();

    // 更新当前槽位的高度差 (持续学习)
    x = ekf_.get_x();
    double new_dz = obs.pos.z() - x[outpost_model::ZC];
    slot_dz_[observed_slot] = new_dz;
    slot_known_[observed_slot] = true;
    current_slot_ = observed_slot;
    current_dz_ = new_dz;

    // 强制速度清零 (前哨站定点旋转)
    if (get_force_zero_velocity()) {
        x[outpost_model::VX] = 0;
        x[outpost_model::VY] = 0;
        ekf_.set_x(x);
    }

    last_update_time_ = timestamp;
}

int OutpostMotion::match_observed_slot(double armor_yaw) const {
    const VectorX x = ekf_.get_x();
    int best_slot = 0;
    double best_diff = std::numeric_limits<double>::max();
    for (int slot = 0; slot < 3; ++slot) {
        const double predicted_yaw =
            x[outpost_model::THETA] + slot * (2.0 * M_PI / 3.0);
        const double diff = std::abs(aimer::math::angle_diff(predicted_yaw, armor_yaw));
        if (diff < best_diff) {
            best_diff = diff;
            best_slot = slot;
        }
    }
    return best_slot;
}

void OutpostMotion::constrain_omega() {
    VectorX x = ekf_.get_x();
    double omega = x[outpost_model::OMEGA];

    // 阶段1: 判断方向 (只需一次)
    if (!omega_sign_determined_) {
        double threshold = get_omega_direction_threshold();
        if (std::abs(omega) > threshold) {
            omega_sign_determined_ = true;
        }
    }

    // 阶段2: 约束角速度 (如果开启锁定)
    if (omega_sign_determined_ && get_lock_omega()) {
        double tolerance = get_omega_tolerance();
        if (std::abs(std::abs(omega) - outpost_model::OMEGA_ABS) > tolerance) {
            x[outpost_model::OMEGA] = std::copysign(outpost_model::OMEGA_ABS, omega);
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

    Q(outpost_model::XC, outpost_model::XC) = q_pos * dt;
    Q(outpost_model::VX, outpost_model::VX) = q_vel * dt;
    Q(outpost_model::YC, outpost_model::YC) = q_pos * dt;
    Q(outpost_model::VY, outpost_model::VY) = q_vel * dt;
    Q(outpost_model::ZC, outpost_model::ZC) = q_pos * dt * 0.1;  // Z 变化很小
    Q(outpost_model::THETA, outpost_model::THETA) = q_theta * dt;
    Q(outpost_model::OMEGA, outpost_model::OMEGA) = q_omega * dt;

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

    R(outpost_model::YAW, outpost_model::YAW) = r_angle * dist_factor;
    R(outpost_model::PITCH, outpost_model::PITCH) = r_angle * dist_factor;
    R(outpost_model::DIS, outpost_model::DIS) = r_dis_k * distance * distance * side_factor;
    R(outpost_model::ARMOR_YAW, outpost_model::ARMOR_YAW) = r_armor_yaw;

    return R;
}

Eigen::Vector3d OutpostMotion::predict_center(double dt) const {
    VectorX x = ekf_.get_x();
    return Eigen::Vector3d(
        x[outpost_model::XC] + x[outpost_model::VX] * dt,
        x[outpost_model::YC] + x[outpost_model::VY] * dt,
        x[outpost_model::ZC]
    );
}

Eigen::Vector3d OutpostMotion::predict_armor_pos(int slot, double dt) const {
    VectorX x = ekf_.get_x();

    double xc = x[outpost_model::XC] + x[outpost_model::VX] * dt;
    double yc = x[outpost_model::YC] + x[outpost_model::VY] * dt;
    double zc = x[outpost_model::ZC];

    double theta = x[outpost_model::THETA] + x[outpost_model::OMEGA] * dt;

    // 槽位角度偏移 (相隔120度)
    double angle_offset = slot * (2.0 * M_PI / 3);
    double armor_theta = theta + angle_offset;

    // 查表获取高度差 (已学习的槽位)
    double dz = slot_dz_[slot];

    double xa = xc - outpost_model::RADIUS * std::cos(armor_theta);
    double ya = yc - outpost_model::RADIUS * std::sin(armor_theta);
    double za = zc + dz;

    return Eigen::Vector3d(xa, ya, za);
}

Eigen::Vector3d OutpostMotion::get_armor_pos() const {
    return predict_armor_pos(current_slot_, 0);
}

Eigen::Vector3d OutpostMotion::get_velocity() const {
    VectorX x = ekf_.get_x();
    return Eigen::Vector3d(x[outpost_model::VX], x[outpost_model::VY], 0);
}

void OutpostMotion::reset() {
    initialized_ = false;
    slot_dz_.fill(0);
    slot_known_.fill(false);
    current_slot_ = 0;
    current_dz_ = 0;
}

}  // namespace autoaim::predictor
