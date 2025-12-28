/**
 * @file spin_motion.cpp
 * @brief 整车旋转模型实现
 */

#include "spin_motion.hpp"

#include <cmath>

#include "aimer/common/math/math.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

// ============================================================================
// EKF 参数 (运行时读取)
// ============================================================================

namespace {

// 辅助函数: 安全读取运行时参数，找不到时返回默认值
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
    return get_double_param("AutoAim.Predictor.SpinEKF.q_pos", 0.1);
}

double get_q_vel() {
    return get_double_param("AutoAim.Predictor.SpinEKF.q_vel", 1.0);
}

double get_q_theta() {
    return get_double_param("AutoAim.Predictor.SpinEKF.q_theta", 0.1);
}

double get_q_omega() {
    return get_double_param("AutoAim.Predictor.SpinEKF.q_omega", 10.0);
}

double get_q_r() {
    return get_double_param("AutoAim.Predictor.SpinEKF.q_r", 0.01);
}

// 观测噪声
double get_r_angle() {
    return get_double_param("AutoAim.Predictor.SpinEKF.r_angle", 0.01);
}

double get_r_dis_k() {
    return get_double_param("AutoAim.Predictor.SpinEKF.r_dis_k", 0.05);
}

double get_r_armor_yaw() {
    return get_double_param("AutoAim.Predictor.SpinEKF.r_armor_yaw", 0.1);
}

// 初始半径
double get_init_r() {
    return get_double_param("AutoAim.Predictor.SpinEKF.init_r", 0.26);
}

// 半径限制
double get_r_min() {
    return get_double_param("AutoAim.Predictor.SpinEKF.r_min", 0.12);
}

double get_r_max() {
    return get_double_param("AutoAim.Predictor.SpinEKF.r_max", 0.4);
}

// 是否强制 Z 轴速度为 0 (PnP 在 Z 方向误差大)
bool get_force_zero_vz() {
    auto ptr = runtime_param::find_param("AutoAim.Predictor.SpinEKF.force_zero_vz");
    if (ptr != nullptr) {
        if (auto* val = std::get_if<bool>(&*ptr)) {
            return *val;
        }
    }
    return true;  // 默认开启
}

// 高度差最大值限制
double get_dz_abs_max() {
    return get_double_param("AutoAim.Predictor.SpinEKF.dz_abs_max", 0.10);
}

}  // namespace

// ============================================================================
// SpinMotion 实现
// ============================================================================

SpinMotion::SpinMotion(int armor_num) : armor_num_(armor_num) {
    another_r_ = get_init_r();
}

void SpinMotion::init(const ArmorData& armor, double timestamp) {
    const auto& obs = armor.observation;

    // 从观测提取
    double xa = obs.pos.x();
    double ya = obs.pos.y();
    double za = obs.pos.z();
    double armor_yaw = obs.z[obs::ARMOR_YAW];

    // 初始半径
    double r = get_init_r();
    another_r_ = r;

    // 从装甲板位置反推旋转中心
    // xa = xc - r * cos(θ)  =>  xc = xa + r * cos(θ)
    // ya = yc - r * sin(θ)  =>  yc = ya + r * sin(θ)
    double xc = xa + r * std::cos(armor_yaw);
    double yc = ya + r * std::sin(armor_yaw);
    double zc = za;

    // 初始化状态
    VectorX x0 = VectorX::Zero();
    x0[spin_model::XC] = xc;
    x0[spin_model::YC] = yc;
    x0[spin_model::ZC] = zc;
    x0[spin_model::THETA] = armor_yaw;
    x0[spin_model::R] = r;
    // 速度初始为 0

    ekf_.init(x0);

    // 重置状态
    dz_ = 0;
    another_dz_ = 0;
    last_yaw_ = armor_yaw;
    spin_level_ = SpinLevel::NONE;
    tracking_armor_id_ = armor.id;  // 记录初始 ID

    last_update_time_ = timestamp;
    initialized_ = true;
}

void SpinMotion::update(const ArmorData& armor, double timestamp) {
    if (!initialized_) {
        init(armor, timestamp);
        return;
    }

    const auto& obs = armor.observation;

    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    // 检测跳变 (用 ID 判断)
    bool jumped = handle_armor_jump(armor);

    // 预测
    SpinCVPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward(predict_func, Q);

    // 观测更新
    double armor_yaw = obs.z[obs::ARMOR_YAW];

    // 连续化 yaw (避免 ±π 跳变)
    VectorX x = ekf_.get_x();
    double theta_pred = x[spin_model::THETA];
    double yaw_continuous = theta_pred + math::angle_diff(theta_pred, armor_yaw);

    // 构建观测向量 (YPD)
    VectorZ z;
    z[spin_model::YAW] = obs.z[obs::YAW];
    z[spin_model::PITCH] = obs.z[obs::PITCH];
    z[spin_model::DIS] = obs.z[obs::DIST];
    z[spin_model::ARMOR_YAW] = yaw_continuous;

    // 观测更新
    SpinMeasure measure_func(dz_);
    MatrixZZ R = build_R(obs.z[obs::DIST], armor.z_to_v());
    ekf_.update_forward(measure_func, z, R);

    // 限制半径范围
    x = ekf_.get_x();
    double r_min = get_r_min();
    double r_max = get_r_max();
    if (x[spin_model::R] < r_min) {
        x[spin_model::R] = r_min;
        ekf_.set_x(x);
    } else if (x[spin_model::R] > r_max) {
        x[spin_model::R] = r_max;
        ekf_.set_x(x);
    }

    // 更新高度差 (当前装甲板相对于中心)
    if (!jumped) {
        dz_ = obs.pos.z() - x[spin_model::ZC];
        // 限制高度差范围
        double dz_max = get_dz_abs_max();
        dz_ = std::clamp(dz_, -dz_max, dz_max);
    }

    // 更新陀螺等级
    update_spin_level();

    // 强制 Z 轴速度为 0 (PnP 在 Z 方向误差大)
    if (get_force_zero_vz()) {
        x = ekf_.get_x();
        x[spin_model::VZ] = 0;
        ekf_.set_x(x);
    }

    last_yaw_ = armor_yaw;
    last_update_time_ = timestamp;
}

bool SpinMotion::handle_armor_jump(const ArmorData& armor) {
    const auto& obs = armor.observation;
    double armor_yaw = obs.z[obs::ARMOR_YAW];
    VectorX x = ekf_.get_x();
    double theta_pred = x[spin_model::THETA];

    // 用 ID 判断是否换了装甲板
    bool id_changed = (armor.id != tracking_armor_id_ && tracking_armor_id_ > 0);

    if (id_changed) {
        // 发生跳变: 换了装甲板

        // 更新朝向角
        x[spin_model::THETA] = theta_pred + math::angle_diff(theta_pred, armor_yaw);

        // 4装甲板: 交换半径和高度差
        if (armor_num_ == 4) {
            std::swap(x[spin_model::R], another_r_);
            std::swap(dz_, another_dz_);
        }

        // 如果位置偏差太大，重置中心位置
        double xa = obs.pos.x();
        double ya = obs.pos.y();
        double za = obs.pos.z();
        double r = x[spin_model::R];
        double theta = x[spin_model::THETA];

        // 从新装甲板位置推算中心
        double xc_obs = xa + r * std::cos(theta);
        double yc_obs = ya + r * std::sin(theta);

        double xc_pred = x[spin_model::XC];
        double yc_pred = x[spin_model::YC];

        double center_diff = std::sqrt(math::sq(xc_obs - xc_pred) + math::sq(yc_obs - yc_pred));

        // 如果中心偏差过大，重置位置和速度
        if (center_diff > 0.5) {
            x[spin_model::XC] = xc_obs;
            x[spin_model::VX] = 0;
            x[spin_model::YC] = yc_obs;
            x[spin_model::VY] = 0;
            x[spin_model::ZC] = za - dz_;
            x[spin_model::VZ] = 0;
        }

        ekf_.set_x(x);
        tracking_armor_id_ = armor.id;  // 更新追踪 ID
        return true;
    }

    // 同一块装甲板，更新 ID (可能是初始化后第一次)
    tracking_armor_id_ = armor.id;
    return false;
}

void SpinMotion::update_spin_level() {
    double omega = std::abs(get_omega());

    // 迟滞阈值
    constexpr double THRESH_LOW = 1.2;
    constexpr double THRESH_HIGH = 3.5;
    constexpr double HYSTERESIS = 0.7;

    switch (spin_level_) {
        case SpinLevel::NONE:
            if (omega > THRESH_LOW) {
                spin_level_ = SpinLevel::LOW;
            }
            break;

        case SpinLevel::LOW:
            if (omega < THRESH_LOW * HYSTERESIS) {
                spin_level_ = SpinLevel::NONE;
            } else if (omega > THRESH_HIGH) {
                spin_level_ = SpinLevel::HIGH;
            }
            break;

        case SpinLevel::HIGH:
            if (omega < THRESH_HIGH * HYSTERESIS) {
                spin_level_ = SpinLevel::LOW;
            }
            break;
    }
}

SpinMotion::MatrixXX SpinMotion::build_Q(double dt) const {
    MatrixXX Q = MatrixXX::Zero();

    double q_pos = get_q_pos();
    double q_vel = get_q_vel();
    double q_theta = get_q_theta();
    double q_omega = get_q_omega();
    double q_r = get_q_r();

    // 位置-速度块 (x, y, z)
    Q(spin_model::XC, spin_model::XC) = q_pos * dt;
    Q(spin_model::VX, spin_model::VX) = q_vel * dt;
    Q(spin_model::YC, spin_model::YC) = q_pos * dt;
    Q(spin_model::VY, spin_model::VY) = q_vel * dt;
    Q(spin_model::ZC, spin_model::ZC) = q_pos * dt;
    Q(spin_model::VZ, spin_model::VZ) = q_vel * dt;

    // 朝向角-角速度块
    Q(spin_model::THETA, spin_model::THETA) = q_theta * dt;
    Q(spin_model::OMEGA, spin_model::OMEGA) = q_omega * dt;

    // 半径
    Q(spin_model::R, spin_model::R) = q_r * dt;

    return Q;
}

SpinMotion::MatrixZZ SpinMotion::build_R(double distance, double z_to_v) const {
    MatrixZZ R = MatrixZZ::Zero();

    double r_angle = get_r_angle();
    double r_dis_k = get_r_dis_k();
    double r_armor_yaw = get_r_armor_yaw();

    // 侧面观看时距离噪声更大 (陀螺模式尤其重要)
    // z_to_v 越大越侧面，log(|z_to_v| + 1) + 1 作为缩放因子
    double side_factor = std::log(std::abs(z_to_v) + 1) + 1;

    // 角度噪声: 与距离无关
    R(spin_model::YAW, spin_model::YAW) = r_angle;
    R(spin_model::PITCH, spin_model::PITCH) = r_angle;

    // 距离噪声: ∝ d² × side_factor
    R(spin_model::DIS, spin_model::DIS) = r_dis_k * distance * distance * side_factor;

    // 装甲板朝向噪声
    R(spin_model::ARMOR_YAW, spin_model::ARMOR_YAW) = r_armor_yaw;

    return R;
}

Eigen::Vector3d SpinMotion::predict_center(double dt) const {
    VectorX x = ekf_.get_x();
    return Eigen::Vector3d(
        x[spin_model::XC] + x[spin_model::VX] * dt,
        x[spin_model::YC] + x[spin_model::VY] * dt,
        x[spin_model::ZC] + x[spin_model::VZ] * dt
    );
}

Eigen::Vector3d SpinMotion::predict_armor_pos(int armor_idx, double dt) const {
    VectorX x = ekf_.get_x();

    // 预测中心
    double xc = x[spin_model::XC] + x[spin_model::VX] * dt;
    double yc = x[spin_model::YC] + x[spin_model::VY] * dt;
    double zc = x[spin_model::ZC] + x[spin_model::VZ] * dt;

    // 预测朝向
    double theta = x[spin_model::THETA] + x[spin_model::OMEGA] * dt;

    // 装甲板角度偏移
    double angle_offset = armor_idx * (2.0 * M_PI / armor_num_);
    double armor_theta = theta + angle_offset;

    // 选择半径和高度差 (4装甲板时交替)
    double r, height_diff;
    if (armor_num_ == 4 && armor_idx % 2 == 1) {
        r = another_r_;
        height_diff = another_dz_;
    } else {
        r = x[spin_model::R];
        height_diff = dz_;
    }

    // 计算装甲板位置
    double xa = xc - r * std::cos(armor_theta);
    double ya = yc - r * std::sin(armor_theta);
    double za = zc + height_diff;

    return Eigen::Vector3d(xa, ya, za);
}

Eigen::Vector3d SpinMotion::get_armor_pos() const {
    return predict_armor_pos(0, 0);
}

Eigen::Vector3d SpinMotion::get_velocity() const {
    VectorX x = ekf_.get_x();
    return Eigen::Vector3d(x[spin_model::VX], x[spin_model::VY], x[spin_model::VZ]);
}

void SpinMotion::reset() {
    initialized_ = false;
    spin_level_ = SpinLevel::NONE;
    dz_ = 0;
    another_dz_ = 0;
    another_r_ = get_init_r();
    tracking_armor_id_ = -1;
}

}  // namespace autoaim::predictor
