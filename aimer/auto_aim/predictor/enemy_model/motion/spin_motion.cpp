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
    double zc = za;

    // 初始化状态
    VectorX x0 = VectorX::Zero();
    x0[spin_model::XC] = xc;
    x0[spin_model::YC] = yc;
    x0[spin_model::ZC] = zc;
    x0[spin_model::THETA] = theta;  // OUTWARD
    x0[spin_model::R] = r;
    // 速度初始为 0

    ekf_.init(x0);

    // 重置状态
    dz_ = 0;
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
    double armor_yaw_inward = obs.z[obs::ARMOR_YAW];
    double armor_yaw_outward = armor_yaw_inward + M_PI;  // 转为 OUTWARD

    // 连续化 theta (避免 ±π 跳变)
    VectorX x = ekf_.get_x();
    double theta_pred = x[spin_model::THETA];
    double theta_continuous = theta_pred + math::angle_diff(theta_pred, armor_yaw_outward);

    // 构建观测向量 (YPD)
    VectorZ z;
    z[spin_model::YAW] = obs.z[obs::YAW];
    z[spin_model::PITCH] = obs.z[obs::PITCH];
    z[spin_model::DIS] = obs.z[obs::DIST];
    z[spin_model::ARMOR_YAW] = theta_continuous;  // OUTWARD

    // 观测更新
    SpinMeasure measure_func(dz_);
    MatrixZZ R = build_R(obs.z[obs::DIST], armor.z_to_v());
    ekf_.update_forward(measure_func, z, R);

    // 限制半径范围
    x = ekf_.get_x();
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_max");
    if (x[spin_model::R] < r_min) {
        x[spin_model::R] = r_min;
        ekf_.set_x(x);
    } else if (x[spin_model::R] > r_max) {
        x[spin_model::R] = r_max;
        ekf_.set_x(x);
    }

    // 更新高度差 (当前装甲板相对于中心)
    x = ekf_.get_x();
    dz_ = obs.pos.z() - x[spin_model::ZC];
    // 限制高度差范围
    double dz_max = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.dz_abs_max");
    dz_ = std::clamp(dz_, -dz_max, dz_max);

    // 更新陀螺等级
    update_spin_level();

    // 强制 Z 轴速度为 0 (PnP 在 Z 方向误差大)
    if (runtime_param::get_param<bool>("AutoAim.Predictor.SpinEKF.force_zero_vz")) {
        x = ekf_.get_x();
        x[spin_model::VZ] = 0;
        ekf_.set_x(x);
    }

    last_yaw_ = armor_yaw_outward;
    last_update_time_ = timestamp;
}

void SpinMotion::update(const std::vector<ArmorData>& armors, double timestamp) {
    if (armors.empty()) return;

    // 单装甲板：用原来的方法
    if (armors.size() == 1) {
        update(armors[0], timestamp);
        return;
    }

    // 多装甲板：参考 rm.cv.fans，不做特殊的几何参数融合，只用主装甲板更新
    const auto& a0 = armors[0];
    const auto& a1 = armors[1];

    if (!initialized_) {
        // 用两块装甲板直接计算初始状态
        Eigen::Vector3d p0 = a0.pos();
        Eigen::Vector3d p1 = a1.pos();
        double yaw0 = a0.observation.z[obs::ARMOR_YAW];
        double yaw1 = a1.observation.z[obs::ARMOR_YAW];

        // 法向量 (从装甲板指向中心, INWARD)
        Eigen::Vector2d n0(std::cos(yaw0), std::sin(yaw0));
        Eigen::Vector2d n1(std::cos(yaw1), std::sin(yaw1));

        // 计算半径: |P0 - P1|_xy / |n1 - n0|
        Eigen::Vector2d dp_xy(p0.x() - p1.x(), p0.y() - p1.y());
        Eigen::Vector2d dn = n1 - n0;
        double dn_norm = dn.norm();
        double r = (dn_norm > 0.1)
            ? dp_xy.norm() / dn_norm
            : runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");
        r = std::clamp(r,
            runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_min"),
            runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_max"));

        // 计算中心: center = armor + r * n (因为 n 指向中心)
        double xc = p0.x() + r * n0.x();
        double yc = p0.y() + r * n0.y();
        double zc = (p0.z() + p1.z()) / 2.0;  // 高度取平均

        // 初始化状态
        VectorX x0 = VectorX::Zero();
        x0[spin_model::XC] = xc;
        x0[spin_model::YC] = yc;
        x0[spin_model::ZC] = zc;
        x0[spin_model::THETA] = yaw0 + M_PI;  // 转为 OUTWARD！
        x0[spin_model::R] = r;

        ekf_.init(x0);

        // 高度差
        dz_ = p0.z() - zc;
        another_dz_ = p1.z() - zc;
        another_r_ = r;  // 平衡步兵两个半径相同

        last_update_time_ = timestamp;
        initialized_ = true;
        return;
    }

    // 已初始化：用主装甲板做 EKF 更新 (不做几何参数融合)
    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    // 预测 (跳变检测已由上层完成)
    SpinCVPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward(predict_func, Q);

    // 用主装甲板做观测更新
    const auto& obs = a0.observation;
    double armor_yaw_inward = obs.z[obs::ARMOR_YAW];
    double armor_yaw_outward = armor_yaw_inward + M_PI;  // 转为 OUTWARD

    VectorX x = ekf_.get_x();
    double theta_pred = x[spin_model::THETA];
    double yaw_continuous = theta_pred + math::angle_diff(theta_pred, armor_yaw_outward);

    VectorZ z;
    z[spin_model::YAW] = obs.z[obs::YAW];
    z[spin_model::PITCH] = obs.z[obs::PITCH];
    z[spin_model::DIS] = obs.z[obs::DIST];
    z[spin_model::ARMOR_YAW] = yaw_continuous;

    SpinMeasure measure_func(dz_);
    MatrixZZ R = build_R(obs.z[obs::DIST], a0.z_to_v(), 2);  // 双块装甲板，噪声更小
    ekf_.update_forward(measure_func, z, R);

    // 后处理 (不做几何参数融合，让 EKF 自己收敛)
    x = ekf_.get_x();
    x[spin_model::R] = std::clamp(x[spin_model::R],
        runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_min"),
        runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_max"));

    // 强制 Z 轴速度为 0
    if (runtime_param::get_param<bool>("AutoAim.Predictor.SpinEKF.force_zero_vz")) {
        x[spin_model::VZ] = 0;
    }
    ekf_.set_x(x);

    update_spin_level();
    last_yaw_ = armor_yaw_outward;
    last_update_time_ = timestamp;
}

void SpinMotion::notify_jump(int jump_index, const ArmorData& new_armor) {
    if (!initialized_ || jump_index <= 0 || jump_index >= armor_num_) return;

    const auto& obs = new_armor.observation;
    // 转换为 OUTWARD
    double armor_yaw_inward = obs.z[obs::ARMOR_YAW];
    double armor_yaw_outward = armor_yaw_inward + M_PI;

    VectorX x = ekf_.get_x();
    double theta_pred = x[spin_model::THETA];

    // 4装甲板: 奇数索引需要交换半径和高度差
    if (armor_num_ == 4 && jump_index % 2 == 1) {
        std::swap(x[spin_model::R], another_r_);
        std::swap(dz_, another_dz_);
    }
    // 3装甲板 (前哨站): 半径固定，不需要交换

    // 更新朝向角为观测值 (连续化, OUTWARD)
    x[spin_model::THETA] = theta_pred + math::angle_diff(theta_pred, armor_yaw_outward);

    // 从新装甲板位置推算中心
    double xa = obs.pos.x();
    double ya = obs.pos.y();
    double za = obs.pos.z();
    double r = x[spin_model::R];
    double theta = x[spin_model::THETA];

    // OUTWARD: center = armor - r * (cos θ, sin θ)
    double xc_obs = xa - r * std::cos(theta);
    double yc_obs = ya - r * std::sin(theta);

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
    const Eigen::Vector3d& observed_pos,
    double observed_theta) const {

    std::vector<Eigen::Vector3d> result;
    result.reserve(armor_num_);

    // 获取几何参数
    VectorX x = ekf_.get_x();
    double r0 = x[spin_model::R];     // 当前装甲板半径
    double r1 = another_r_;           // 另一个半径
    double z0_diff = dz_;             // 当前高度差
    double z1_diff = another_dz_;     // 另一个高度差

    // 从观测装甲板反推中心 (OUTWARD): center = armor - r * (cos θ, sin θ)
    double xc = observed_pos.x() - r0 * std::cos(observed_theta);
    double yc = observed_pos.y() - r0 * std::sin(observed_theta);
    double zc = observed_pos.z() - z0_diff;  // 中心 z = 装甲板 z - 高度差

    // 计算所有装甲板位置
    double angle_step = 2.0 * M_PI / armor_num_;
    double current_r = r0;
    double current_dz = z0_diff;

    for (int i = 0; i < armor_num_; ++i) {
        double theta_i = observed_theta + i * angle_step;

        // 装甲板位置 (OUTWARD): armor = center + r * (cos θ, sin θ)
        double xa = xc + current_r * std::cos(theta_i);
        double ya = yc + current_r * std::sin(theta_i);
        double za = zc + current_dz;

        result.emplace_back(xa, ya, za);

        // 4装甲板时交替切换半径和高度差
        if (armor_num_ == 4) {
            std::swap(current_r, r1);
            current_dz = -current_dz;  // 高度差取反
            std::swap(current_dz, z1_diff);
        }
    }

    return result;
}

void SpinMotion::reset() {
    initialized_ = false;
    spin_level_ = SpinLevel::NONE;
    dz_ = 0;
    another_dz_ = 0;
    another_r_ = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");
}

}  // namespace autoaim::predictor
