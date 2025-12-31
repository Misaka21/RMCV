/**
 * @file spin_motion.cpp
 * @brief 整车旋转模型实现
 */

#include "spin_motion.hpp"

#include <cmath>

#include "aimer/common/math/math.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

// ============================================================================
// SpinMotion 实现
// ============================================================================

SpinMotion::SpinMotion(int armor_num) : armor_num_(armor_num) {
    another_r_ = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");
}

void SpinMotion::init(const ArmorData& armor, double timestamp) {
    const auto& obs = armor.observation;

    // 从观测提取
    double xa = obs.pos.x();
    double ya = obs.pos.y();
    double za = obs.pos.z();

    // 装甲板朝向 (PnP 输出的是装甲板面朝方向，即 INWARD)
    // 需要转换为 OUTWARD: 从中心指向装甲板
    double armor_yaw_inward = obs.z[obs::ARMOR_YAW];
    double theta = armor_yaw_inward + M_PI;  // 转为 OUTWARD

    // 初始半径
    double r = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");
    another_r_ = r;

    // 从装甲板位置反推旋转中心
    // OUTWARD: center = armor - r * (cos θ, sin θ)
    double xc = xa - r * std::cos(theta);
    double yc = ya - r * std::sin(theta);
    double zc = za;  // 初始时 dz = 0，所以 zc = za

    // 初始化状态 (10维)
    VectorX x0 = VectorX::Zero();
    x0[spin_model::XC] = xc;
    x0[spin_model::YC] = yc;
    x0[spin_model::ZC] = zc;
    x0[spin_model::THETA] = theta;  // OUTWARD
    x0[spin_model::R] = r;
    x0[spin_model::DZ] = 0;  // 初始高度差为 0
    // 速度初始为 0

    ekf_.init(x0);

    // 重置状态
    another_dz_ = 0;
    last_yaw_ = theta;  // 保存 OUTWARD 角度
    spin_level_ = SpinLevel::NONE;

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

    // 预测 (跳变检测已由上层完成)
    SpinCVPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward(predict_func, Q);

    // 观测的装甲板朝向 (INWARD → OUTWARD)
    double orient_yaw = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD

    // 获取 EKF 内部预测的观测值 (用于连续化)
    SpinMeasure measure_func;
    VectorZ inner_z;
    VectorX x = ekf_.get_x();
    double x_arr[spin_model::N_X], z_arr[spin_model::N_Z];
    for (int i = 0; i < spin_model::N_X; ++i) x_arr[i] = x[i];
    measure_func(x_arr, z_arr);
    for (int i = 0; i < spin_model::N_Z; ++i) inner_z[i] = z_arr[i];

    // 连续化 (把观测调整到离 EKF 预测最近，避免 ±π 跳变)
    VectorZ z;
    z[spin_model::YAW] = math::get_closest_angle(obs.z[obs::YAW], inner_z[spin_model::YAW]);
    z[spin_model::PITCH] = obs.z[obs::PITCH];
    z[spin_model::DIS] = obs.z[obs::DIST];
    z[spin_model::ARMOR_YAW] = math::get_closest_angle(orient_yaw, inner_z[spin_model::ARMOR_YAW]);

    // 观测更新
    MatrixZZ R = build_R(obs.z[obs::DIST], armor.z_to_v());
    ekf_.update_forward(measure_func, z, R);

    // 后处理：限制半径和高度差范围
    x = ekf_.get_x();
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_max");
    double dz_max = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.dz_abs_max");

    x[spin_model::R] = std::clamp(x[spin_model::R], r_min, r_max);
    x[spin_model::DZ] = std::clamp(x[spin_model::DZ], -dz_max, dz_max);
    another_r_ = std::clamp(another_r_, r_min, r_max);  // 同时约束 another_r_

    // 强制 Z 轴速度为 0 (PnP 在 Z 方向误差大)
    if (runtime_param::get_param<bool>("AutoAim.Predictor.SpinEKF.force_zero_vz")) {
        x[spin_model::VZ] = 0;
    }
    ekf_.set_x(x);

    // 更新陀螺等级
    update_spin_level();

    last_yaw_ = orient_yaw;
    last_update_time_ = timestamp;
}

void SpinMotion::update(const std::vector<ArmorData>& armors, double timestamp) {
    if (armors.empty()) return;

    // 单装甲板：用原来的方法
    if (armors.size() == 1) {
        update(armors[0], timestamp);
        return;
    }

    // ==================== 双装甲板处理 ====================
    // 参考 rm.cv.fans/LmtdMotion 设计：
    // - 不用射线交点法计算半径（对 PnP 误差敏感，会抖）
    // - 半径让 EKF 自己通过观测慢慢收敛
    // - 双装甲板时只是降低观测噪声（朝向角更可信）

    const auto& primary = armors[0];  // 用第一个装甲板（z_to_v 更小，更正对）

    // ==================== 初始化 ====================
    if (!initialized_) {
        double init_r = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");
        double primary_theta = primary.observation.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD

        // 从装甲板反推中心
        double xc = primary.pos().x() - init_r * std::cos(primary_theta);
        double yc = primary.pos().y() - init_r * std::sin(primary_theta);

        VectorX x0 = VectorX::Zero();
        x0[spin_model::XC] = xc;
        x0[spin_model::YC] = yc;
        x0[spin_model::ZC] = primary.pos().z();  // 初始 dz=0
        x0[spin_model::THETA] = primary_theta;
        x0[spin_model::R] = init_r;
        x0[spin_model::DZ] = 0;

        ekf_.init(x0);

        another_r_ = init_r;  // 两个半径初始相同
        another_dz_ = 0;

        last_update_time_ = timestamp;
        initialized_ = true;
        return;
    }

    // ==================== EKF 更新 ====================
    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    SpinCVPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward(predict_func, Q);

    // 用主装甲板做观测更新
    const auto& obs = primary.observation;
    double orient_yaw = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD

    // 获取 EKF 内部预测的观测值 (用于连续化)
    SpinMeasure measure_func;
    VectorZ inner_z;
    VectorX x = ekf_.get_x();
    double x_arr[spin_model::N_X], z_arr[spin_model::N_Z];
    for (int i = 0; i < spin_model::N_X; ++i) x_arr[i] = x[i];
    measure_func(x_arr, z_arr);
    for (int i = 0; i < spin_model::N_Z; ++i) inner_z[i] = z_arr[i];

    // 连续化 (把观测调整到离 EKF 预测最近，避免 ±π 跳变)
    VectorZ z;
    z[spin_model::YAW] = math::get_closest_angle(obs.z[obs::YAW], inner_z[spin_model::YAW]);
    z[spin_model::PITCH] = obs.z[obs::PITCH];
    z[spin_model::DIS] = obs.z[obs::DIST];
    z[spin_model::ARMOR_YAW] = math::get_closest_angle(orient_yaw, inner_z[spin_model::ARMOR_YAW]);

    // 双装甲板时观测噪声更小（朝向角更可信）
    MatrixZZ R = build_R(obs.z[obs::DIST], primary.z_to_v(), 2);
    ekf_.update_forward(measure_func, z, R);

    // 后处理
    x = ekf_.get_x();
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_max");
    double dz_max = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.dz_abs_max");

    x[spin_model::R] = std::clamp(x[spin_model::R], r_min, r_max);
    x[spin_model::DZ] = std::clamp(x[spin_model::DZ], -dz_max, dz_max);
    another_r_ = std::clamp(another_r_, r_min, r_max);  // 同时约束 another_r_

    if (runtime_param::get_param<bool>("AutoAim.Predictor.SpinEKF.force_zero_vz")) {
        x[spin_model::VZ] = 0;
    }
    ekf_.set_x(x);

    update_spin_level();
    last_yaw_ = orient_yaw;
    last_update_time_ = timestamp;
}

void SpinMotion::notify_jump(int jump_index, const ArmorData& new_armor) {
    if (!initialized_ || jump_index <= 0 || jump_index >= armor_num_) return;

    const auto& obs = new_armor.observation;
    // 转换为 OUTWARD
    double armor_yaw_inward = obs.z[obs::ARMOR_YAW];
    double new_theta = armor_yaw_inward + M_PI;  // OUTWARD

    VectorX x = ekf_.get_x();
    double old_theta = x[spin_model::THETA];

    debug::print(debug::PrintMode::DEBUG, "SpinMotion",
        "Jump: index={}, theta={:.1f}° -> {:.1f}°",
        jump_index, old_theta * 180.0 / M_PI, new_theta * 180.0 / M_PI);

    // 4装甲板: 奇数索引需要交换半径和高度差
    if (armor_num_ == 4 && jump_index % 2 == 1) {
        debug::print(debug::PrintMode::DEBUG, "SpinMotion",
            "Before swap: r={:.3f}, another_r={:.3f}, dz={:.3f}, another_dz={:.3f}",
            x[spin_model::R], another_r_, x[spin_model::DZ], another_dz_);

        std::swap(x[spin_model::R], another_r_);
        std::swap(x[spin_model::DZ], another_dz_);

        debug::print(debug::PrintMode::DEBUG, "SpinMotion",
            "After swap: r={:.3f}, another_r={:.3f}, dz={:.3f}, another_dz={:.3f}",
            x[spin_model::R], another_r_, x[spin_model::DZ], another_dz_);
    }
    // 3装甲板 (前哨站): 半径固定，不需要交换

    // rm.cv.fans 关键设计：跳变时只更新 theta，不更新中心位置 xc, yc, zc
    // 让 EKF 通过后续观测自己收敛中心位置，避免 PnP 误差直接影响中心
    x[spin_model::THETA] = new_theta;

    debug::print(debug::PrintMode::DEBUG, "SpinMotion",
        "Jump: theta={:.1f}°, r={:.3f} (center unchanged, let EKF converge)",
        new_theta * 180.0 / M_PI, x[spin_model::R]);

    ekf_.set_x(x);
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

    double q_pos = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.q_pos");
    double q_vel = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.q_vel");
    double q_theta = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.q_theta");
    double q_omega = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.q_omega");
    double q_r = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.q_r");
    double q_dz = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.q_dz");

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

    // 半径和高度差 (rm.cv.fans: q_r 不乘 dt)
    Q(spin_model::R, spin_model::R) = q_r;
    Q(spin_model::DZ, spin_model::DZ) = q_dz;

    return Q;
}

SpinMotion::MatrixZZ SpinMotion::build_R(double distance, double z_to_v, int observed_armor_count) const {
    MatrixZZ R = MatrixZZ::Zero();

    double r_angle = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_angle");
    double r_dis_k = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_dis_k");

    // 根据观测到的装甲板数量选择朝向噪声
    // - 单块: PnP 姿态估计误差大，噪声设大
    // - 双块: 几何关系互相验证，噪声设小
    double r_armor_yaw = (observed_armor_count >= 2)
        ? runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_armor_yaw_double")
        : runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_armor_yaw_single");

    // 角度噪声: 与距离无关
    R(spin_model::YAW, spin_model::YAW) = r_angle;
    R(spin_model::PITCH, spin_model::PITCH) = r_angle;

    // 距离噪声: ∝ dis^4 (与 rm.cv.fans 一致)
    R(spin_model::DIS, spin_model::DIS) = r_dis_k * std::pow(distance, 4.0);

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
        height_diff = x[spin_model::DZ];  // 从状态向量获取
    }

    // 计算装甲板位置 (OUTWARD): armor = center + r * (cos θ, sin θ)
    double xa = xc + r * std::cos(armor_theta);
    double ya = yc + r * std::sin(armor_theta);
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

std::vector<Eigen::Vector3d> SpinMotion::compute_all_armors_from_observation(
    const Eigen::Vector3d& /* observed_pos */,
    double /* observed_theta */) const {
    // rm.cv.fans 原版设计: 直接从 EKF 状态生成所有装甲板
    // 不需要从观测反推！更稳定，避免飘走

    std::vector<Eigen::Vector3d> result;
    result.reserve(armor_num_);

    // 直接用 predict_armor_pos，和 rm.cv.fans 的 predict_armors 一样
    for (int i = 0; i < armor_num_; ++i) {
        result.push_back(predict_armor_pos(i, 0));  // dt=0，当前时刻
    }

    return result;
}

void SpinMotion::reset() {
    initialized_ = false;
    spin_level_ = SpinLevel::NONE;
    another_dz_ = 0;
    another_r_ = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");
}

}  // namespace autoaim::predictor
