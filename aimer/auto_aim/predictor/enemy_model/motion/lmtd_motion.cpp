/**
 * @file lmtd_motion.cpp
 * @brief LMTD 整车旋转模型实现
 *
 * 关键 trick (来自 rm.cv.fans):
 * 1. 内部跳变检测 - 通过 tracked_armor_id 判断
 * 2. 装甲板选择 - 优先保持追踪 + keep_tracking_area_ratio
 * 3. credit 时间判断 - 超时重新 init
 * 4. 位置 yaw 也要连续化 - 防止 ±π 跳变
 * 5. 距离观测噪声 ∝ dis^4 - 远距离噪声放大
 */

#include "lmtd_motion.hpp"

#include <cmath>
#include <cfloat>

#include "aimer/common/math/math.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

// ============================================================================
// LmtdMotion 实现
// ============================================================================

LmtdMotion::LmtdMotion(int armor_num) : armor_num_(armor_num) {
    another_r_ = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.init_r");
}

void LmtdMotion::init(const ArmorData& armor, double timestamp) {
    const auto& obs = armor.observation;

    // 装甲板位置 (世界系)
    double xa = obs.pos.x();
    double ya = obs.pos.y();
    double za = obs.pos.z();

    // 装甲板朝向 (从 ArmorObserver 输出的是 INWARD，需要转 OUTWARD)
    double armor_yaw_inward = obs.z[obs::ARMOR_YAW];
    double theta = armor_yaw_inward + M_PI;  // 转为 OUTWARD

    // 初始半径
    double r = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.init_r");
    another_r_ = r;

    // 从装甲板反推中心 (OUTWARD: center = armor - r * (cos θ, sin θ))
    double xc = xa - r * std::cos(theta);
    double yc = ya - r * std::sin(theta);

    // 初始化状态
    VectorX x0 = VectorX::Zero();
    x0[lmtd_model::XC] = xc;
    x0[lmtd_model::YC] = yc;
    x0[lmtd_model::ZA] = za;  // 装甲板 z，不是中心 z!
    x0[lmtd_model::THETA] = theta;
    x0[lmtd_model::R] = r;

    ekf_.init(x0);

    // 重置状态
    dz_ = 0;
    spin_level_ = SpinLevel::NONE;
    top_level_ = 0;
    tracked_armor_id_ = armor.id;  // 记录追踪的装甲板 ID

    last_update_time_ = timestamp;
    predict_t_ = timestamp;
    update_t_ = timestamp;
    initialized_ = true;
}

bool LmtdMotion::credit(double current_time) const {
    double credit_dt = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.credit_dt");
    return current_time - update_t_ <= credit_dt;
}

bool LmtdMotion::detect_and_handle_jump(const ArmorData& armor) {
    // 如果 ID 相同，没有跳变
    if (armor.id == tracked_armor_id_) {
        return false;
    }

    // ID 不同，发生跳变！
    // 计算跳了几块装甲板

    const auto& obs = armor.observation;
    double new_orient = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD
    double new_za = obs.pos.z();

    VectorX x = ekf_.get_x();
    double state_theta = x[lmtd_model::THETA];
    double old_za = x[lmtd_model::ZA];

    // 遍历所有可能的装甲板位置，找最接近观测角度的
    int most_like_index = 0;
    double min_yaw_diff = DBL_MAX;

    for (int i = 0; i < armor_num_; ++i) {
        double possible_theta = math::normalize_angle(state_theta + i * (2.0 * M_PI / armor_num_));
        double yaw_diff = std::abs(math::angle_diff(possible_theta, new_orient));

        if (yaw_diff < min_yaw_diff) {
            min_yaw_diff = yaw_diff;
            most_like_index = i;
        }
    }

    if (most_like_index == 0) {
        // 没有实际跳变，只是 ID 变了
        tracked_armor_id_ = armor.id;
        return false;
    }

    // 真的跳变了
    debug::print(debug::PrintMode::DEBUG, "LmtdMotion",
        "Jump detected: {} -> {}, index={}", tracked_armor_id_, armor.id, most_like_index);

    // 4装甲板: 奇数跳变交换半径和高度差
    if (armor_num_ == 4 && most_like_index % 2 == 1) {
        std::swap(x[lmtd_model::R], another_r_);

        // LMTD 关键: dz = old_za - new_za
        dz_ = old_za - new_za;

        // 限制高度差
        double dz_max = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.dz_abs_max");
        dz_ = std::clamp(dz_, -dz_max, dz_max);

        // 直接设为新装甲板的 z (避免累积误差)
        x[lmtd_model::ZA] = new_za;
    }

    // 直接用观测的朝向角 (不用 possible_theta，因为角速度慢时不准)
    // 参考 rm.cv.fans: 只更新 theta，不更新 xc, yc，让 EKF 自己收敛
    x[lmtd_model::THETA] = new_orient;

    ekf_.set_x(x);
    tracked_armor_id_ = armor.id;

    return true;
}

int LmtdMotion::select_armor_to_track(const std::vector<ArmorData>& armors) const {
    if (armors.size() == 1) {
        return 0;
    }

    // 找最大面积和当前追踪 ID
    double max_area = -DBL_MAX;
    int max_area_idx = 0;
    double tracked_area = 0;
    int tracked_idx = -1;

    for (size_t i = 0; i < armors.size(); ++i) {
        double area = math::get_area(armors[i].observation.pts);
        if (area > max_area) {
            max_area = area;
            max_area_idx = static_cast<int>(i);
        }
        if (armors[i].id == tracked_armor_id_) {
            tracked_idx = static_cast<int>(i);
            tracked_area = area;
        }
    }

    // 优先保持追踪当前 ID (防止反复横跳)
    double keep_ratio = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.keep_tracking_area_ratio");
    if (tracked_idx >= 0 && tracked_area >= keep_ratio * max_area) {
        return tracked_idx;
    }

    return max_area_idx;
}

void LmtdMotion::update(const ArmorData& armor, double timestamp) {
    // credit 检查: 超时则重新 init
    if (!initialized_ || !credit(timestamp)) {
        init(armor, timestamp);
        return;
    }

    double dt = timestamp - predict_t_;
    if (dt <= 0) return;

    // 预测
    LmtdPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward(predict_func, Q);
    predict_t_ = timestamp;

    // 内部跳变检测 (LMTD 核心 trick)
    detect_and_handle_jump(armor);

    // 构建观测
    const auto& obs = armor.observation;
    double orient_yaw = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD

    // 获取内部预测的观测值 (用于连续化)
    LmtdMeasure measure_func;
    VectorZ inner_z;
    VectorX x = ekf_.get_x();
    double x_arr[lmtd_model::N_X], z_arr[lmtd_model::N_Z];
    for (int i = 0; i < lmtd_model::N_X; ++i) x_arr[i] = x[i];
    measure_func(x_arr, z_arr);
    for (int i = 0; i < lmtd_model::N_Z; ++i) inner_z[i] = z_arr[i];

    // 连续化 (LMTD trick: 位置 yaw 也要连续化!)
    VectorZ z;
    z[lmtd_model::YAW] = inner_z[lmtd_model::YAW] +
        math::angle_diff(inner_z[lmtd_model::YAW], obs.z[obs::YAW]);
    z[lmtd_model::PITCH] = obs.z[obs::PITCH];
    z[lmtd_model::DIS] = obs.z[obs::DIST];
    z[lmtd_model::ORIENT_YAW] = inner_z[lmtd_model::ORIENT_YAW] +
        math::angle_diff(inner_z[lmtd_model::ORIENT_YAW], orient_yaw);

    // 观测更新
    MatrixZZ R = build_R(obs.z[obs::DIST], armor.z_to_v());
    ekf_.update_forward(measure_func, z, R);

    // 后处理
    x = ekf_.get_x();

    // 限制半径范围
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_max");
    x[lmtd_model::R] = std::clamp(x[lmtd_model::R], r_min, r_max);

    // 强制 Z 轴速度为 0
    if (runtime_param::get_param<bool>("AutoAim.Predictor.LmtdEKF.force_zero_vz")) {
        x[lmtd_model::VZ] = 0;
    }

    ekf_.set_x(x);

    update_spin_level();
    update_t_ = timestamp;
    last_update_time_ = timestamp;
}

void LmtdMotion::update(const std::vector<ArmorData>& armors, double timestamp) {
    if (armors.empty()) return;

    // 选择要追踪的装甲板
    int track_idx = select_armor_to_track(armors);
    const ArmorData& primary = armors[track_idx];

    if (armors.size() == 1) {
        update(primary, timestamp);
        return;
    }

    // 多装甲板模式
    if (!initialized_ || !credit(timestamp)) {
        // 用两块装甲板直接计算初始状态
        const auto& a0 = armors[0];
        const auto& a1 = armors[1];

        Eigen::Vector3d p0 = a0.pos();
        Eigen::Vector3d p1 = a1.pos();
        double theta0 = a0.observation.z[obs::ARMOR_YAW] + M_PI;
        double theta1 = a1.observation.z[obs::ARMOR_YAW] + M_PI;

        Eigen::Vector2d n0(std::cos(theta0), std::sin(theta0));
        Eigen::Vector2d n1(std::cos(theta1), std::sin(theta1));

        Eigen::Vector2d dp_xy(p0.x() - p1.x(), p0.y() - p1.y());
        Eigen::Vector2d dn = n0 - n1;
        double dn_norm = dn.norm();
        double r = (dn_norm > 0.1)
            ? dp_xy.norm() / dn_norm
            : runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.init_r");
        r = std::clamp(r,
            runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_min"),
            runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_max"));

        double xc = p0.x() - r * n0.x();
        double yc = p0.y() - r * n0.y();

        VectorX x0 = VectorX::Zero();
        x0[lmtd_model::XC] = xc;
        x0[lmtd_model::YC] = yc;
        x0[lmtd_model::ZA] = primary.pos().z();
        x0[lmtd_model::THETA] = primary.observation.z[obs::ARMOR_YAW] + M_PI;
        x0[lmtd_model::R] = r;

        ekf_.init(x0);

        dz_ = p0.z() - p1.z();
        another_r_ = r;
        tracked_armor_id_ = primary.id;

        last_update_time_ = timestamp;
        predict_t_ = timestamp;
        update_t_ = timestamp;
        initialized_ = true;
        return;
    }

    // 已初始化：EKF 更新
    double dt = timestamp - predict_t_;
    if (dt <= 0) return;

    LmtdPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward(predict_func, Q);
    predict_t_ = timestamp;

    // 跳变检测 (参考 rm.cv.fans: 双装甲板时也需要)
    detect_and_handle_jump(primary);

    // 观测更新 (用主装甲板)
    const auto& obs = primary.observation;
    double orient_yaw = obs.z[obs::ARMOR_YAW] + M_PI;

    LmtdMeasure measure_func;
    VectorZ inner_z;
    VectorX x = ekf_.get_x();
    double x_arr[lmtd_model::N_X], z_arr[lmtd_model::N_Z];
    for (int i = 0; i < lmtd_model::N_X; ++i) x_arr[i] = x[i];
    measure_func(x_arr, z_arr);
    for (int i = 0; i < lmtd_model::N_Z; ++i) inner_z[i] = z_arr[i];

    VectorZ z;
    z[lmtd_model::YAW] = inner_z[lmtd_model::YAW] +
        math::angle_diff(inner_z[lmtd_model::YAW], obs.z[obs::YAW]);
    z[lmtd_model::PITCH] = obs.z[obs::PITCH];
    z[lmtd_model::DIS] = obs.z[obs::DIST];
    z[lmtd_model::ORIENT_YAW] = inner_z[lmtd_model::ORIENT_YAW] +
        math::angle_diff(inner_z[lmtd_model::ORIENT_YAW], orient_yaw);

    MatrixZZ R = build_R(obs.z[obs::DIST], primary.z_to_v(), 2);
    ekf_.update_forward(measure_func, z, R);

    // 利用两块装甲板更新几何参数
    const auto& a0 = armors[0];
    const auto& a1 = armors[1];
    Eigen::Vector3d p0 = a0.pos();
    Eigen::Vector3d p1 = a1.pos();
    double theta0 = a0.observation.z[obs::ARMOR_YAW] + M_PI;
    double theta1 = a1.observation.z[obs::ARMOR_YAW] + M_PI;

    Eigen::Vector2d n0(std::cos(theta0), std::sin(theta0));
    Eigen::Vector2d n1(std::cos(theta1), std::sin(theta1));
    Eigen::Vector2d dp_xy(p0.x() - p1.x(), p0.y() - p1.y());
    Eigen::Vector2d dn = n0 - n1;
    double dn_norm = dn.norm();

    if (dn_norm > 0.1) {
        double r_measured = dp_xy.norm() / dn_norm;
        r_measured = std::clamp(r_measured,
            runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_min"),
            runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_max"));

        x = ekf_.get_x();
        constexpr double ALPHA = 0.3;
        double xc_measured = (p0.x() - r_measured * n0.x() + p1.x() - r_measured * n1.x()) / 2.0;
        double yc_measured = (p0.y() - r_measured * n0.y() + p1.y() - r_measured * n1.y()) / 2.0;

        x[lmtd_model::XC] = (1 - ALPHA) * x[lmtd_model::XC] + ALPHA * xc_measured;
        x[lmtd_model::YC] = (1 - ALPHA) * x[lmtd_model::YC] + ALPHA * yc_measured;
        x[lmtd_model::R] = (1 - ALPHA) * x[lmtd_model::R] + ALPHA * r_measured;

        dz_ = p0.z() - p1.z();

        ekf_.set_x(x);
    }

    // 后处理
    x = ekf_.get_x();
    x[lmtd_model::R] = std::clamp(x[lmtd_model::R],
        runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_min"),
        runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_max"));

    if (runtime_param::get_param<bool>("AutoAim.Predictor.LmtdEKF.force_zero_vz")) {
        x[lmtd_model::VZ] = 0;
    }
    ekf_.set_x(x);

    update_spin_level();
    update_t_ = timestamp;
    last_update_time_ = timestamp;
}

void LmtdMotion::update_spin_level() {
    double omega = std::abs(get_omega());

    double activate_1 = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.top1_activate_w");
    double deactivate_1 = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.top1_deactivate_w");
    double activate_2 = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.top2_activate_w");
    double deactivate_2 = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.top2_deactivate_w");

    if (top_level_ == 0) {
        if (omega >= activate_1) top_level_ = 1;
    } else if (top_level_ == 1) {
        if (omega < deactivate_1) top_level_ = 0;
        else if (omega >= activate_2) top_level_ = 2;
    } else {
        if (omega < deactivate_2) top_level_ = 1;
    }

    switch (top_level_) {
        case 0: spin_level_ = SpinLevel::NONE; break;
        case 1: spin_level_ = SpinLevel::LOW; break;
        case 2: spin_level_ = SpinLevel::HIGH; break;
    }
}

LmtdMotion::MatrixXX LmtdMotion::build_Q(double dt) const {
    MatrixXX Q = MatrixXX::Zero();

    double q_pos = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.q_pos");
    double q_vel = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.q_vel");
    double q_theta = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.q_theta");
    double q_omega = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.q_omega");
    double q_r = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.q_r");

    // LMTD 风格: Q = q * dt
    Q(lmtd_model::XC, lmtd_model::XC) = q_pos * dt;
    Q(lmtd_model::VX, lmtd_model::VX) = q_vel * dt;
    Q(lmtd_model::YC, lmtd_model::YC) = q_pos * dt;
    Q(lmtd_model::VY, lmtd_model::VY) = q_vel * dt;
    Q(lmtd_model::ZA, lmtd_model::ZA) = q_pos * dt;
    Q(lmtd_model::VZ, lmtd_model::VZ) = q_vel * dt;
    Q(lmtd_model::THETA, lmtd_model::THETA) = q_theta * dt;
    Q(lmtd_model::OMEGA, lmtd_model::OMEGA) = q_omega * dt;
    Q(lmtd_model::R, lmtd_model::R) = q_r;  // 半径噪声不乘 dt

    return Q;
}

LmtdMotion::MatrixZZ LmtdMotion::build_R(double distance, double z_to_v, int observed_armor_count) const {
    MatrixZZ R = MatrixZZ::Zero();

    double r_angle = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_angle");
    double r_dis_1m = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_dis_1m");

    double r_orient = (observed_armor_count >= 2)
        ? runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_orient_double")
        : runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_orient_single");

    R(lmtd_model::YAW, lmtd_model::YAW) = r_angle;
    R(lmtd_model::PITCH, lmtd_model::PITCH) = r_angle;

    // LMTD trick: 距离噪声 ∝ dis^4
    R(lmtd_model::DIS, lmtd_model::DIS) = r_dis_1m * std::pow(distance, 4.0);

    R(lmtd_model::ORIENT_YAW, lmtd_model::ORIENT_YAW) = r_orient;

    return R;
}

Eigen::Vector3d LmtdMotion::predict_center(double dt) const {
    VectorX x = ekf_.get_x();
    return Eigen::Vector3d(
        x[lmtd_model::XC] + x[lmtd_model::VX] * dt,
        x[lmtd_model::YC] + x[lmtd_model::VY] * dt,
        x[lmtd_model::ZA] + x[lmtd_model::VZ] * dt
    );
}

Eigen::Vector3d LmtdMotion::predict_armor_pos(int armor_idx, double dt) const {
    VectorX x = ekf_.get_x();

    double xc = x[lmtd_model::XC] + x[lmtd_model::VX] * dt;
    double yc = x[lmtd_model::YC] + x[lmtd_model::VY] * dt;
    double theta = x[lmtd_model::THETA] + x[lmtd_model::OMEGA] * dt;

    double angle_offset = armor_idx * (2.0 * M_PI / armor_num_);
    double armor_theta = theta + angle_offset;

    double r = x[lmtd_model::R];
    double za = x[lmtd_model::ZA] + x[lmtd_model::VZ] * dt;
    double another_r = another_r_;
    double dz = dz_;

    for (int i = 0; i < armor_idx; ++i) {
        if (armor_num_ == 4) {
            std::swap(r, another_r);
            za += dz;
            dz = -dz;
        }
    }

    // OUTWARD: armor = center + r * (cos θ, sin θ)
    double xa = xc + r * std::cos(armor_theta);
    double ya = yc + r * std::sin(armor_theta);

    return Eigen::Vector3d(xa, ya, za);
}

Eigen::Vector3d LmtdMotion::get_armor_pos() const {
    return predict_armor_pos(0, 0);
}

Eigen::Vector3d LmtdMotion::get_armor_velocity() const {
    return lmtd_state_to_armor_velocity(ekf_.get_x());
}

Eigen::Vector3d LmtdMotion::get_center_velocity() const {
    return lmtd_state_to_center_velocity(ekf_.get_x());
}

std::vector<Eigen::Vector3d> LmtdMotion::compute_all_armors_from_observation(
    const Eigen::Vector3d& observed_pos,
    double observed_theta) const {

    std::vector<Eigen::Vector3d> result;
    result.reserve(armor_num_);

    VectorX x = ekf_.get_x();
    double r0 = x[lmtd_model::R];
    double r1 = another_r_;

    // OUTWARD: center = armor - r * (cos θ, sin θ)
    double xc = observed_pos.x() - r0 * std::cos(observed_theta);
    double yc = observed_pos.y() - r0 * std::sin(observed_theta);

    double angle_step = 2.0 * M_PI / armor_num_;
    double current_r = r0;
    double current_z = observed_pos.z();
    double current_dz = dz_;

    for (int i = 0; i < armor_num_; ++i) {
        double theta_i = observed_theta + i * angle_step;

        double xa = xc + current_r * std::cos(theta_i);
        double ya = yc + current_r * std::sin(theta_i);

        result.emplace_back(xa, ya, current_z);

        if (armor_num_ == 4) {
            std::swap(current_r, r1);
            current_z += current_dz;
            current_dz = -current_dz;
        }
    }

    return result;
}

void LmtdMotion::reset() {
    initialized_ = false;
    spin_level_ = SpinLevel::NONE;
    top_level_ = 0;
    dz_ = 0;
    tracked_armor_id_ = -1;
    another_r_ = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.init_r");
}

}  // namespace autoaim::predictor
