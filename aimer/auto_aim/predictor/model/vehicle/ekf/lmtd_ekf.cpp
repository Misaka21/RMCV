/**
 * @file lmtd_motion.cpp
 * @brief LMTD 整车旋转模型实现
 *
 * 关键 trick (来自 rm.cv.fans):
 * 1. 内部跳变检测 - 通过 tracked_detector_id 判断同一物理板
 * 2. 装甲板选择 - 优先保持追踪 + keep_tracking_area_ratio
 * 3. credit 时间判断 - 超时重新 init
 * 4. 位置 yaw 也要连续化 - 防止 ±π 跳变
 * 5. 距离观测噪声 ∝ dis^4 - 远距离噪声放大
 */

#include "lmtd_ekf.hpp"

#include <cmath>
#include <cfloat>

#include "aimer/common/math/math.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/rerun/rmcv_rerun.hpp"

namespace autoaim::predictor {

// ============================================================================
// LmtdMotion 实现
// ============================================================================

LmtdMotion::LmtdMotion(int armor_num) : armor_num_(armor_num) {
    // another_r_ 在 init() 中从 runtime_param 读取，支持热更新
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
    tracked_state_id_ = 0;
    tracked_detector_id_ = armor.id;

    last_update_time_ = timestamp;
    predict_t_ = timestamp;
    update_t_ = timestamp;
    initialized_ = true;
}

bool LmtdMotion::credit(double current_time) const {
    double credit_dt = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.credit_dt");
    return current_time - update_t_ <= credit_dt;
}

bool LmtdMotion::detect_and_handle_jump(const ArmorData& armor, int& out_tracked_state_id) {
    // 返回新的追踪装甲板序号，但不在这里更新成员变量
    out_tracked_state_id = tracked_state_id_;

    // detector ID 相同，认为是同一物理装甲板
    if (armor.id == tracked_detector_id_) {
        return false;
    }

    // ID 不同，可能发生跳变
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
        double possible_theta = aimer::math::normalize_angle(state_theta + i * (2.0 * M_PI / armor_num_));
        double yaw_diff = std::abs(aimer::math::angle_diff(possible_theta, new_orient));

        if (yaw_diff < min_yaw_diff) {
            min_yaw_diff = yaw_diff;
            most_like_index = i;
        }
    }

    int new_state_id = (tracked_state_id_ + most_like_index) % armor_num_;
    out_tracked_state_id = new_state_id;

    if (most_like_index == 0) {
        // 没有实际跳变，只是 ID 变了
        return false;
    }

    // 真的跳变了
    debug::print(debug::PrintMode::DEBUG, "LmtdMotion",
        "Jump detected: {} -> {}, index={}, state_theta={:.1f}°, new_orient={:.1f}°",
        tracked_state_id_, armor.id, most_like_index,
        state_theta * 180.0 / M_PI, new_orient * 180.0 / M_PI);

    // 4装甲板: 奇数跳变交换半径和高度差
    if (armor_num_ == 4 && most_like_index % 2 == 1) {
        debug::print(debug::PrintMode::DEBUG, "LmtdMotion",
            "Before swap: r={:.3f}, another_r={:.3f}, old_za={:.3f}, new_za={:.3f}, dz={:.3f}",
            x[lmtd_model::R], another_r_, old_za, new_za, dz_);

        std::swap(x[lmtd_model::R], another_r_);

        // LMTD 关键: dz = old_za - new_za (和 rm.cv.fans 一致，只 clamp 不平滑)
        dz_ = old_za - new_za;
        double dz_max = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.dz_abs_max");
        dz_ = std::clamp(dz_, -dz_max, dz_max);

        // 直接设为新装甲板的 z (避免累积误差)
        x[lmtd_model::ZA] = new_za;

        // 关键修复：r 交换后，需要根据新 r 和观测位置反推新中心
        // 否则用旧中心 + 新 r 计算的装甲板位置会跳变
        // center = armor_pos - r * (cos θ, sin θ)  (OUTWARD)
        double new_r = x[lmtd_model::R];
        x[lmtd_model::XC] = obs.pos.x() - new_r * std::cos(new_orient);
        x[lmtd_model::YC] = obs.pos.y() - new_r * std::sin(new_orient);

        debug::print(debug::PrintMode::DEBUG, "LmtdMotion",
            "After swap: r={:.3f}, another_r={:.3f}, za={:.3f}, dz={:.3f}",
            x[lmtd_model::R], another_r_, x[lmtd_model::ZA], dz_);
    } else {
        debug::print(debug::PrintMode::DEBUG, "LmtdMotion",
            "Even jump (index={}): no swap, r={:.3f}, another_r={:.3f}",
            most_like_index, x[lmtd_model::R], another_r_);
    }

    // 更新 theta
    x[lmtd_model::THETA] = new_orient;

    debug::print(debug::PrintMode::DEBUG, "LmtdMotion",
        "Jump: theta={:.1f}°, r={:.3f} (center unchanged, let EKF converge)",
        new_orient * 180.0 / M_PI, x[lmtd_model::R]);

    ekf_.set_x(x);

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
        double area = aimer::math::get_area(armors[i].observation.pts);
        if (area > max_area) {
            max_area = area;
            max_area_idx = static_cast<int>(i);
        }
        if (armors[i].id == tracked_detector_id_) {
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
    // 参考 rm.cv.fans: tracked_state_id 在 EKF 更新后才更新
    int new_tracked_state_id;
    detect_and_handle_jump(armor, new_tracked_state_id);

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
    // 把观测角度调整到离内部预测最近，避免 ±π 跳变
    VectorZ z;
    z[lmtd_model::YAW] = aimer::math::get_closest_angle(obs.z[obs::YAW], inner_z[lmtd_model::YAW]);
    z[lmtd_model::PITCH] = obs.z[obs::PITCH];
    z[lmtd_model::DIS] = obs.z[obs::DIST];
    z[lmtd_model::ORIENT_YAW] = aimer::math::get_closest_angle(orient_yaw, inner_z[lmtd_model::ORIENT_YAW]);

    // 观测更新
    MatrixZZ R = build_R(obs.z[obs::DIST], armor.z_to_v());
    ekf_.update_forward(measure_func, z, R);

    // 参考 rm.cv.fans: EKF 更新后才更新追踪状态
    tracked_state_id_ = new_tracked_state_id;
    tracked_detector_id_ = armor.id;

    // 后处理
    x = ekf_.get_x();

    // 限制半径范围 (两个半径都要 clamp，防止 swap 时越界)
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_max");
    x[lmtd_model::R] = std::clamp(x[lmtd_model::R], r_min, r_max);
    another_r_ = std::clamp(another_r_, r_min, r_max);

    // 强制 Z 轴速度为 0
    if (runtime_param::get_param<bool>("AutoAim.Predictor.LmtdEKF.force_zero_vz")) {
        x[lmtd_model::VZ] = 0;
    }

    ekf_.set_x(x);

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

    // ==================== 双装甲板处理 ====================
    // rm.cv.fans 原版设计：不用射线交点法计算半径
    // 半径让 EKF 自己通过观测慢慢收敛
    // 双装甲板时只用 fit_double_z_to_l 优化朝向角，提高观测精度

    // ==================== 初始化或更新 ====================
    if (!initialized_ || !credit(timestamp)) {
        // 使用默认半径初始化
        double init_r = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.init_r");

        // primary 的朝向角 (OUTWARD)
        double primary_theta = primary.observation.z[obs::ARMOR_YAW] + M_PI;

        // 从 primary 装甲板反推中心
        double xc = primary.pos().x() - init_r * std::cos(primary_theta);
        double yc = primary.pos().y() - init_r * std::sin(primary_theta);

        VectorX x0 = VectorX::Zero();
        x0[lmtd_model::XC] = xc;
        x0[lmtd_model::YC] = yc;
        x0[lmtd_model::ZA] = primary.pos().z();
        x0[lmtd_model::THETA] = primary_theta;
        x0[lmtd_model::R] = init_r;
        another_r_ = init_r;  // rm.cv.fans: 两个半径都初始化为默认值

        // rm.cv.fans: 初始化时 dz = 0，通过跳变时学习高度差
        dz_ = 0;

        ekf_.init(x0);

        tracked_state_id_ = 0;
        tracked_detector_id_ = primary.id;

        debug::print(debug::PrintMode::DEBUG, "LmtdMotion",
            "Init with dual armors: r={:.3f}, another_r={:.3f}, dz={:.3f}",
            init_r, another_r_, dz_);

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

    // 跳变检测
    int new_tracked_state_id;
    detect_and_handle_jump(primary, new_tracked_state_id);

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

    // 连续化 (把观测调整到离预测最近)
    VectorZ z;
    z[lmtd_model::YAW] = aimer::math::get_closest_angle(obs.z[obs::YAW], inner_z[lmtd_model::YAW]);
    z[lmtd_model::PITCH] = obs.z[obs::PITCH];
    z[lmtd_model::DIS] = obs.z[obs::DIST];
    z[lmtd_model::ORIENT_YAW] = aimer::math::get_closest_angle(orient_yaw, inner_z[lmtd_model::ORIENT_YAW]);

    MatrixZZ R = build_R(obs.z[obs::DIST], primary.z_to_v(), 2);
    ekf_.update_forward(measure_func, z, R);

    // EKF 更新后才更新追踪状态
    tracked_state_id_ = new_tracked_state_id;
    tracked_detector_id_ = primary.id;

    // rm.cv.fans 原版设计：不做几何融合，让 EKF 自己收敛半径
    // 半径通过 EKF 观测模型自然收敛

    // 后处理 (两个半径都要 clamp)
    x = ekf_.get_x();
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.r_max");
    x[lmtd_model::R] = std::clamp(x[lmtd_model::R], r_min, r_max);
    another_r_ = std::clamp(another_r_, r_min, r_max);

    if (runtime_param::get_param<bool>("AutoAim.Predictor.LmtdEKF.force_zero_vz")) {
        x[lmtd_model::VZ] = 0;
    }
    ekf_.set_x(x);

    update_t_ = timestamp;
    last_update_time_ = timestamp;
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
    const Eigen::Vector3d& /* observed_pos */,
    double /* observed_theta */,
    int /* observed_id */) const {
    // 直接调用新接口
    return compute_all_armors(0);
}

std::vector<Eigen::Vector3d> LmtdMotion::compute_all_armors(double dt) const {
    std::vector<Eigen::Vector3d> result;
    result.reserve(armor_num_);

    for (int i = 0; i < armor_num_; ++i) {
        result.push_back(predict_armor_pos(i, dt));
    }

    return result;
}

void LmtdMotion::log_state(const std::string& prefix) const {
    VectorX x = ekf_.get_x();

    rr::scalar(prefix + "/xc", x[lmtd_model::XC]);
    rr::scalar(prefix + "/yc", x[lmtd_model::YC]);
    rr::scalar(prefix + "/za", x[lmtd_model::ZA]);
    rr::scalar(prefix + "/vx", x[lmtd_model::VX]);
    rr::scalar(prefix + "/vy", x[lmtd_model::VY]);
    rr::scalar(prefix + "/vz", x[lmtd_model::VZ]);
    rr::scalar(prefix + "/theta", x[lmtd_model::THETA] * 57.3);
    rr::scalar(prefix + "/omega", x[lmtd_model::OMEGA]);
    rr::scalar(prefix + "/r", x[lmtd_model::R]);
    rr::scalar(prefix + "/another_r", another_r_);
    rr::scalar(prefix + "/dz", dz_);
    rr::scalar(prefix + "/tracked_id", tracked_state_id_);
    rr::scalar(prefix + "/tracked_detector_id", tracked_detector_id_);
}

void LmtdMotion::reset() {
    initialized_ = false;
    dz_ = 0;
    tracked_state_id_ = 0;
    tracked_detector_id_ = -1;
    another_r_ = runtime_param::get_param<double>("AutoAim.Predictor.LmtdEKF.init_r");
}

}  // namespace autoaim::predictor
