/**
 * @file spin_motion.cpp
 * @brief 整车旋转模型实现
 */

#include "spin_ekf.hpp"

#include <cmath>

#include "aimer/common/math/math.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/plotter/plotter.hpp"

namespace autoaim::predictor {

// ============================================================================
// SpinMotion 实现
// ============================================================================

SpinMotion::SpinMotion(int armor_num) : armor_num_(armor_num) {
    // another_r_ 在 init() 中从 runtime_param 读取，支持热更新
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
    tracked_armor_id_ = armor.id;  // 记录追踪的装甲板 ID

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

    // 1. 预测
    SpinCVPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward_scaled(predict_func, Q);

    // 2. 跳变检测 (在 predict 之后，观测更新之前)
    int new_tracked_id;
    detect_and_handle_jump(armor, new_tracked_id);

    // 3. 观测的装甲板朝向 (INWARD → OUTWARD)
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
    z[spin_model::YAW] = aimer::math::get_closest_angle(obs.z[obs::YAW], inner_z[spin_model::YAW]);
    z[spin_model::PITCH] = obs.z[obs::PITCH];
    z[spin_model::DIS] = obs.z[obs::DIST];
    z[spin_model::ARMOR_YAW] = aimer::math::get_closest_angle(orient_yaw, inner_z[spin_model::ARMOR_YAW]);

    // 构建重置状态 (门限检查失败时使用)
    VectorX reset_state = build_reset_state(armor);

    // 读取门限参数
    double chi2_threshold = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.Gating.chi2_threshold");
    int64_t max_reject = runtime_param::get_param<int64_t>("AutoAim.Predictor.SpinEKF.Gating.max_reject");
    double q_scale_increase = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.Gating.q_scale_increase");
    double q_scale_decay = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.Gating.q_scale_decay");

    // 4. 带门限检查的观测更新
    MatrixZZ R = build_R(obs.z[obs::DIST], armor.z_to_v());
    auto status = ekf_.update_forward_gated(
        measure_func, z, R, reset_state,
        chi2_threshold, max_reject, q_scale_increase, q_scale_decay
    );

    // 5. 更新追踪 ID (在 EKF 更新之后，与 LmtdMotion 一致)
    tracked_armor_id_ = new_tracked_id;

    // 处理更新结果
    if (status == aimer::filter::UpdateStatus::RESET) {
        // EKF 已重置，同时重置外部状态
        double init_r = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");
        another_r_ = init_r;
        another_dz_ = 0;
        spin_level_ = SpinLevel::NONE;
        tracked_armor_id_ = armor.id;  // 重置时也更新 ID
        debug::print(debug::PrintMode::WARNING, "SpinMotion",
            "EKF reset due to {} consecutive rejections", max_reject);
    } else if (status == aimer::filter::UpdateStatus::REJECTED) {
        debug::print(debug::PrintMode::DEBUG, "SpinMotion",
            "Observation rejected, q_scale={:.2f}, reject_count={}",
            ekf_.get_q_scale(), ekf_.get_reject_count());
    }

    // 后处理：限制半径和高度差范围
    x = ekf_.get_x();
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.r_max");
    double dz_max = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.dz_abs_max");

    x[spin_model::R] = std::clamp(x[spin_model::R], r_min, r_max);
    x[spin_model::DZ] = std::clamp(x[spin_model::DZ], -dz_max, dz_max);
    another_r_ = std::clamp(another_r_, r_min, r_max);

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
        tracked_armor_id_ = primary.id;  // 记录追踪 ID

        last_update_time_ = timestamp;
        initialized_ = true;
        return;
    }

    // ==================== EKF 更新 ====================
    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    // 1. 预测
    SpinCVPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward_scaled(predict_func, Q);

    // 2. 跳变检测 (在 predict 之后)
    int new_tracked_id;
    detect_and_handle_jump(primary, new_tracked_id);

    // 3. 用主装甲板做观测更新
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
    z[spin_model::YAW] = aimer::math::get_closest_angle(obs.z[obs::YAW], inner_z[spin_model::YAW]);
    z[spin_model::PITCH] = obs.z[obs::PITCH];
    z[spin_model::DIS] = obs.z[obs::DIST];
    z[spin_model::ARMOR_YAW] = aimer::math::get_closest_angle(orient_yaw, inner_z[spin_model::ARMOR_YAW]);

    // 构建重置状态 (门限检查失败时使用)
    VectorX reset_state = build_reset_state(primary);

    // 读取门限参数
    double chi2_threshold = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.Gating.chi2_threshold");
    int64_t max_reject = runtime_param::get_param<int64_t>("AutoAim.Predictor.SpinEKF.Gating.max_reject");
    double q_scale_increase = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.Gating.q_scale_increase");
    double q_scale_decay = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.Gating.q_scale_decay");

    // 双装甲板时观测噪声更小（朝向角更可信）
    MatrixZZ R = build_R(obs.z[obs::DIST], primary.z_to_v(), 2);
    auto status = ekf_.update_forward_gated(
        measure_func, z, R, reset_state,
        chi2_threshold, max_reject, q_scale_increase, q_scale_decay
    );

    // 处理更新结果
    if (status == aimer::filter::UpdateStatus::RESET) {
        double init_r = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");
        another_r_ = init_r;
        another_dz_ = 0;
        spin_level_ = SpinLevel::NONE;
        tracked_armor_id_ = primary.id;  // 重置时也更新 ID
        debug::print(debug::PrintMode::WARNING, "SpinMotion",
            "EKF reset due to {} consecutive rejections (dual armor)", max_reject);
    } else if (status == aimer::filter::UpdateStatus::REJECTED) {
        debug::print(debug::PrintMode::DEBUG, "SpinMotion",
            "Observation rejected (dual armor), q_scale={:.2f}", ekf_.get_q_scale());
    }

    // 更新追踪 ID (在 EKF 更新之后，与 LmtdMotion 一致)
    tracked_armor_id_ = new_tracked_id;

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

bool SpinMotion::detect_and_handle_jump(const ArmorData& armor, int& out_tracked_id) {
    out_tracked_id = armor.id;

    // 如果 ID 相同，没有跳变
    if (armor.id == tracked_armor_id_) {
        return false;
    }

    // ID 不同，可能发生跳变
    const auto& obs = armor.observation;
    double new_orient = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD

    VectorX x = ekf_.get_x();
    double state_theta = x[spin_model::THETA];

    // 遍历所有可能的装甲板位置，找最接近观测角度的
    int most_like_index = 0;
    double min_yaw_diff = std::abs(aimer::math::angle_diff(state_theta, new_orient));

    double angle_step = 2.0 * M_PI / armor_num_;
    for (int i = 1; i < armor_num_; ++i) {
        double possible_theta = aimer::math::normalize_angle(state_theta + i * angle_step);
        double yaw_diff = std::abs(aimer::math::angle_diff(possible_theta, new_orient));

        if (yaw_diff < min_yaw_diff) {
            min_yaw_diff = yaw_diff;
            most_like_index = i;
        }
    }

    if (most_like_index == 0) {
        // 没有实际跳变，只是 ID 变了
        return false;
    }

    // 真的跳变了
    debug::print(debug::PrintMode::DEBUG, "SpinMotion",
        "Jump detected: {} -> {}, index={}, state_theta={:.1f}°, new_orient={:.1f}°",
        tracked_armor_id_, armor.id, most_like_index,
        state_theta * 180.0 / M_PI, new_orient * 180.0 / M_PI);

    // 4装甲板: 奇数跳变交换半径和高度差
    if (armor_num_ == 4 && most_like_index % 2 == 1) {
        debug::print(debug::PrintMode::DEBUG, "SpinMotion",
            "Before swap: r={:.3f}, another_r={:.3f}, dz={:.3f}, another_dz={:.3f}",
            x[spin_model::R], another_r_, x[spin_model::DZ], another_dz_);

        std::swap(x[spin_model::R], another_r_);
        std::swap(x[spin_model::DZ], another_dz_);

        debug::print(debug::PrintMode::DEBUG, "SpinMotion",
            "After swap: r={:.3f}, another_r={:.3f}, dz={:.3f}, another_dz={:.3f}",
            x[spin_model::R], another_r_, x[spin_model::DZ], another_dz_);

        // 关键修复：r 交换后，需要根据新 r 和观测位置反推新中心
        // 否则用旧中心 + 新 r 计算的装甲板位置会跳变
        // center = armor_pos - r * (cos θ, sin θ)  (OUTWARD)
        double new_r = x[spin_model::R];
        x[spin_model::XC] = obs.pos.x() - new_r * std::cos(new_orient);
        x[spin_model::YC] = obs.pos.y() - new_r * std::sin(new_orient);
        x[spin_model::ZC] = obs.pos.z() - x[spin_model::DZ];
    }

    // 更新 theta
    x[spin_model::THETA] = new_orient;

    debug::print(debug::PrintMode::DEBUG, "SpinMotion",
        "Jump: theta={:.1f}°, r={:.3f} (center unchanged, let EKF converge)",
        new_orient * 180.0 / M_PI, x[spin_model::R]);

    ekf_.set_x(x);

    return true;
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
    // 直接调用新接口
    return compute_all_armors(0);
}

std::vector<Eigen::Vector3d> SpinMotion::compute_all_armors(double dt) const {
    std::vector<Eigen::Vector3d> result;
    result.reserve(armor_num_);

    for (int i = 0; i < armor_num_; ++i) {
        result.push_back(predict_armor_pos(i, dt));
    }

    return result;
}

void SpinMotion::output_to_plotter(const std::string& prefix) const {
    VectorX x = ekf_.get_x();

    plotter::add(prefix + "/xc", x[spin_model::XC]);
    plotter::add(prefix + "/yc", x[spin_model::YC]);
    plotter::add(prefix + "/zc", x[spin_model::ZC]);
    plotter::add(prefix + "/vx", x[spin_model::VX]);
    plotter::add(prefix + "/vy", x[spin_model::VY]);
    plotter::add(prefix + "/vz", x[spin_model::VZ]);
    plotter::add(prefix + "/theta", x[spin_model::THETA] * 57.3);  // 转换为度
    plotter::add(prefix + "/omega", x[spin_model::OMEGA]);
    plotter::add(prefix + "/r", x[spin_model::R]);
    plotter::add(prefix + "/dz", x[spin_model::DZ]);
    plotter::add(prefix + "/another_r", another_r_);
    plotter::add(prefix + "/another_dz", another_dz_);
    plotter::add(prefix + "/spin_level", static_cast<int>(spin_level_));
    plotter::add(prefix + "/tracked_id", tracked_armor_id_);
}

void SpinMotion::reset() {
    initialized_ = false;
    spin_level_ = SpinLevel::NONE;
    another_dz_ = 0;
    another_r_ = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");
    tracked_armor_id_ = -1;  // 重置追踪 ID
}

SpinMotion::VectorX SpinMotion::build_reset_state(const ArmorData& armor) const {
    const auto& obs = armor.observation;

    // 从观测提取
    double xa = obs.pos.x();
    double ya = obs.pos.y();
    double za = obs.pos.z();

    // 装甲板朝向 (PnP 输出的是 INWARD，转为 OUTWARD)
    double armor_yaw_inward = obs.z[obs::ARMOR_YAW];
    double theta = armor_yaw_inward + M_PI;

    // 初始半径
    double r = runtime_param::get_param<double>("AutoAim.Predictor.SpinEKF.init_r");

    // 从装甲板位置反推旋转中心 (OUTWARD: center = armor - r * (cos θ, sin θ))
    double xc = xa - r * std::cos(theta);
    double yc = ya - r * std::sin(theta);
    double zc = za;  // 初始 dz = 0

    // 构建重置状态
    VectorX x0 = VectorX::Zero();
    x0[spin_model::XC] = xc;
    x0[spin_model::YC] = yc;
    x0[spin_model::ZC] = zc;
    x0[spin_model::THETA] = theta;
    x0[spin_model::R] = r;
    x0[spin_model::DZ] = 0;
    // 速度保持为 0

    return x0;
}

}  // namespace autoaim::predictor
