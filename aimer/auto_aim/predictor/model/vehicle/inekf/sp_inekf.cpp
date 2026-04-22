/**
 * @file sp_inekf.cpp
 * @brief SP 整车旋转模型 InEKF 实现
 */

#include "sp_inekf.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "aimer/common/math/math.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/rerun/rmcv_rerun.hpp"

namespace autoaim::predictor {

SpInekfMotion::SpInekfMotion(int armor_num) : armor_num_(armor_num) {}

// ============================================================================
// 初始化
// ============================================================================

void SpInekfMotion::init(const ArmorData& armor, double timestamp) {
    const auto& obs = armor.observation;

    double xa = obs.pos.x();
    double ya = obs.pos.y();
    double za = obs.pos.z();

    double armor_yaw_inward = obs.z[obs::ARMOR_YAW];
    double armor_theta = armor_yaw_inward + M_PI;  // OUTWARD

    double r = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.init_r");

    double xc = xa - r * std::cos(armor_theta);
    double yc = ya - r * std::sin(armor_theta);

    aimer::math::SE2 X0(armor_theta, Eigen::Vector2d(xc, yc));
    VectorB b0 = VectorB::Zero();
    b0[sp_inekf_model::ZC] = za;
    b0[sp_inekf_model::R] = r;

    ekf_.init(X0, b0);

    tracked_armor_id_ = 0;
    last_detector_id_ = armor.id;
    last_update_time_ = timestamp;
    initialized_ = true;

    debug::print(debug::PrintMode::DEBUG, "SpInekfMotion",
        "Init: armor.id={}, theta={:.1f}°, r={:.3f}, center=({:.2f},{:.2f},{:.2f})",
        armor.id, armor_theta * 180.0 / M_PI, r, xc, yc, za);
}

// ============================================================================
// 装甲板匹配
// ============================================================================

int SpInekfMotion::match_armor(const ArmorData& armor) const {
    if (armor.id == last_detector_id_ && armor.id >= 0) {
        return tracked_armor_id_;
    }

    const auto& obs = armor.observation;
    double obs_yaw = obs.z[obs::YAW];
    double obs_armor_yaw = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD

    auto X = ekf_.get_X();
    auto b = ekf_.get_b();

    struct Candidate {
        int id;
        double distance;
        double angle_error;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(armor_num_);

    for (int i = 0; i < armor_num_; ++i) {
        Eigen::Vector3d pred_pos = h_armor_xyz(X, b, i);
        double dist = (pred_pos - obs.pos).norm();

        double pred_armor_yaw = X.theta + i * (2.0 * M_PI / armor_num_);
        double pred_yaw = std::atan2(pred_pos.y(), pred_pos.x());

        double yaw_diff = std::abs(aimer::math::angle_diff(obs_yaw, pred_yaw));
        double orient_diff = std::abs(aimer::math::angle_diff(obs_armor_yaw, pred_armor_yaw));
        double angle_error = yaw_diff + orient_diff;

        candidates.push_back({i, dist, angle_error});
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const auto& a, const auto& b) { return a.distance < b.distance; });

    int num_to_check = std::min(3, static_cast<int>(candidates.size()));
    int best_id = candidates[0].id;
    double min_error = candidates[0].angle_error;
    double best_dist = candidates[0].distance;

    for (int i = 1; i < num_to_check; ++i) {
        if (candidates[i].angle_error < min_error) {
            min_error = candidates[i].angle_error;
            best_id = candidates[i].id;
            best_dist = candidates[i].distance;
        }
    }

    double switch_angle_gain = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.match_switch_min_angle_gain");
    double switch_dist_gain = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.match_switch_min_dist_gain");
    double tracked_max_dist = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.match_tracked_max_dist");
    int prev_id = tracked_armor_id_;
    if (best_id != prev_id) {
        for (const auto& c : candidates) {
            if (c.id != prev_id) continue;
            bool tracked_reasonable = (c.distance <= tracked_max_dist);
            bool angle_much_better = (c.angle_error - min_error) > switch_angle_gain;
            bool dist_much_better = (c.distance - best_dist) > switch_dist_gain;
            bool new_has_clear_advantage = angle_much_better || dist_much_better;
            if (tracked_reasonable && !new_has_clear_advantage) {
                return prev_id;
            }
            break;
        }
    }

    return best_id;
}

Eigen::Vector3d SpInekfMotion::h_armor_xyz(const aimer::math::SE2& X, const VectorB& b, int id) const {
    bool use_l_h = (armor_num_ == 4) && (id % 2 == 1);
    double r_actual = use_l_h ? (b[sp_inekf_model::R] + b[sp_inekf_model::L]) : b[sp_inekf_model::R];
    double z_actual = use_l_h ? (b[sp_inekf_model::ZC] + b[sp_inekf_model::H]) : b[sp_inekf_model::ZC];
    double angle = X.theta + id * (2.0 * M_PI / armor_num_);

    return Eigen::Vector3d(
        X.t.x() + r_actual * std::cos(angle),
        X.t.y() + r_actual * std::sin(angle),
        z_actual
    );
}

// ============================================================================
// 核心更新逻辑
// ============================================================================

void SpInekfMotion::update(const ArmorData& armor, double timestamp) {
    if (!initialized_) {
        init(armor, timestamp);
        return;
    }

    const auto& obs = armor.observation;

    constexpr double Z_TO_V_MAX = 1.6;
    if (std::abs(armor.z_to_v()) > Z_TO_V_MAX) {
        debug::print(debug::PrintMode::WARNING, "SpInekfMotion",
            "Reject observation: z_to_v={:.3f} out of range", armor.z_to_v());
        return;
    }

    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    // 1. 左不变预测
    auto b = ekf_.get_b();
    Eigen::Vector3d u(b[sp_inekf_model::OMEGA], b[sp_inekf_model::VX], b[sp_inekf_model::VY]);
    MatrixXX Q = build_Q(dt);
    ekf_.predict(u, dt, Q);

    // 2. 匹配装甲板 ID
    int matched_id = match_armor(armor);
    tracked_armor_id_ = matched_id;
    last_detector_id_ = armor.id;

    // 3. 观测更新
    double orient_yaw = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD

    auto X = ekf_.get_X();
    SpInekfMeasure measure_func(X.t.x(), X.t.y(), X.theta, matched_id, armor_num_);

    VectorZ inner_z;
    {
        double x_arr[sp_inekf_model::TOTAL_DIM] = {};
        double z_arr[sp_inekf_model::MEAS_DIM] = {};
        measure_func(x_arr, z_arr);
        for (int i = 0; i < sp_inekf_model::MEAS_DIM; ++i) inner_z[i] = z_arr[i];
    }

    VectorZ z;
    z[sp_inekf_model::YAW] = aimer::math::get_closest_angle(obs.z[obs::YAW], inner_z[sp_inekf_model::YAW]);
    z[sp_inekf_model::PITCH] = obs.z[obs::PITCH];
    z[sp_inekf_model::DIS] = obs.z[obs::DIST];
    z[sp_inekf_model::ARMOR_YAW] = aimer::math::get_closest_angle(orient_yaw, inner_z[sp_inekf_model::ARMOR_YAW]);

    double predicted_armor_yaw_inward = inner_z[sp_inekf_model::ARMOR_YAW] - M_PI;
    double predicted_z_to_v = std::abs(aimer::math::angle_diff(
        predicted_armor_yaw_inward, inner_z[sp_inekf_model::YAW]));

    double chi2_threshold = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.chi2_threshold");
    int64_t max_reject = runtime_param::get_param<int64_t>("AutoAim.Predictor.SpEKF.Gating.max_reject");
    double q_scale_increase = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.q_scale_increase");
    double q_scale_decay = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.q_scale_decay");

    aimer::math::SE2 reset_X;
    VectorB reset_b = VectorB::Zero();
    {
        double init_r = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.init_r");
        double xc = obs.pos.x() - init_r * std::cos(orient_yaw);
        double yc = obs.pos.y() - init_r * std::sin(orient_yaw);
        double theta_state0 = orient_yaw - matched_id * (2.0 * M_PI / armor_num_);
        reset_X = aimer::math::SE2(theta_state0, Eigen::Vector2d(xc, yc));
        reset_b[sp_inekf_model::ZC] = obs.pos.z();
        reset_b[sp_inekf_model::R] = init_r;
    }

    MatrixZZ R = build_R(obs.z[obs::DIST], predicted_z_to_v, 1);
    auto status = ekf_.update_forward_gated(
        measure_func, z, R, reset_X, reset_b,
        chi2_threshold, max_reject, q_scale_increase, q_scale_decay
    );

    if (status == aimer::filter::InEKFUpdateStatus::RESET) {
        tracked_armor_id_ = 0;
        debug::print(debug::PrintMode::WARNING, "SpInekfMotion",
            "InEKF reset due to {} consecutive rejections", max_reject);
    }

    // 后处理
    b = ekf_.get_b();
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_max");
    double l_abs_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.l_abs_max");
    double h_abs_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.h_abs_max");

    b[sp_inekf_model::R] = std::clamp(b[sp_inekf_model::R], r_min, r_max);
    b[sp_inekf_model::L] = std::clamp(b[sp_inekf_model::L], -l_abs_max, l_abs_max);
    b[sp_inekf_model::H] = std::clamp(b[sp_inekf_model::H], -h_abs_max, h_abs_max);

    if (runtime_param::get_param<bool>("AutoAim.Predictor.SpEKF.force_zero_vz")) {
        b[sp_inekf_model::VZ] = 0;
    }
    ekf_.set_b(b);

    last_update_time_ = timestamp;
}

void SpInekfMotion::update(const std::vector<ArmorData>& armors, double timestamp) {
    if (armors.empty()) return;

    const auto& primary = [&]() -> const ArmorData& {
        if (armors.size() == 1) return armors[0];

        double keep_ratio = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.keep_tracking_area_ratio");
        double max_area = 0;
        int max_area_idx = 0;
        double tracked_area = 0;
        int tracked_idx = -1;

        for (size_t i = 0; i < armors.size(); ++i) {
            double area = aimer::math::get_area(armors[i].observation.pts);
            if (area > max_area) {
                max_area = area;
                max_area_idx = static_cast<int>(i);
            }
            if (armors[i].id == last_detector_id_) {
                tracked_idx = static_cast<int>(i);
                tracked_area = area;
            }
        }

        if (tracked_idx >= 0) {
            if (tracked_area >= keep_ratio * max_area) {
                return armors[tracked_idx];
            }
            return armors[max_area_idx];
        }
        return armors[max_area_idx];
    }();

    if (armors.size() == 1) {
        update(primary, timestamp);
        return;
    }

    constexpr double Z_TO_V_MAX = 1.6;
    if (std::abs(primary.z_to_v()) > Z_TO_V_MAX) {
        debug::print(debug::PrintMode::WARNING, "SpInekfMotion",
            "Reject dual observation: z_to_v={:.3f} out of range", primary.z_to_v());
        return;
    }

    if (!initialized_) {
        init(primary, timestamp);
        return;
    }

    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    // 1. 预测
    auto b = ekf_.get_b();
    Eigen::Vector3d u(b[sp_inekf_model::OMEGA], b[sp_inekf_model::VX], b[sp_inekf_model::VY]);
    MatrixXX Q = build_Q(dt);
    ekf_.predict(u, dt, Q);

    // 2. 匹配
    int matched_id = match_armor(primary);
    tracked_armor_id_ = matched_id;
    last_detector_id_ = primary.id;

    // 3. 观测更新
    const auto& obs = primary.observation;
    double orient_yaw = obs.z[obs::ARMOR_YAW] + M_PI;

    auto X = ekf_.get_X();
    SpInekfMeasure measure_func(X.t.x(), X.t.y(), X.theta, matched_id, armor_num_);

    VectorZ inner_z;
    {
        double x_arr[sp_inekf_model::TOTAL_DIM] = {};
        double z_arr[sp_inekf_model::MEAS_DIM] = {};
        measure_func(x_arr, z_arr);
        for (int i = 0; i < sp_inekf_model::MEAS_DIM; ++i) inner_z[i] = z_arr[i];
    }

    VectorZ z;
    z[sp_inekf_model::YAW] = aimer::math::get_closest_angle(obs.z[obs::YAW], inner_z[sp_inekf_model::YAW]);
    z[sp_inekf_model::PITCH] = obs.z[obs::PITCH];
    z[sp_inekf_model::DIS] = obs.z[obs::DIST];
    z[sp_inekf_model::ARMOR_YAW] = aimer::math::get_closest_angle(orient_yaw, inner_z[sp_inekf_model::ARMOR_YAW]);

    double predicted_armor_yaw_inward = inner_z[sp_inekf_model::ARMOR_YAW] - M_PI;
    double predicted_z_to_v = std::abs(aimer::math::angle_diff(
        predicted_armor_yaw_inward, inner_z[sp_inekf_model::YAW]));

    double chi2_threshold = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.chi2_threshold");
    int64_t max_reject = runtime_param::get_param<int64_t>("AutoAim.Predictor.SpEKF.Gating.max_reject");
    double q_scale_increase = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.q_scale_increase");
    double q_scale_decay = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.q_scale_decay");

    aimer::math::SE2 reset_X;
    VectorB reset_b = VectorB::Zero();
    {
        double init_r = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.init_r");
        double xc = obs.pos.x() - init_r * std::cos(orient_yaw);
        double yc = obs.pos.y() - init_r * std::sin(orient_yaw);
        double theta_state0 = orient_yaw - matched_id * (2.0 * M_PI / armor_num_);
        reset_X = aimer::math::SE2(theta_state0, Eigen::Vector2d(xc, yc));
        reset_b[sp_inekf_model::ZC] = obs.pos.z();
        reset_b[sp_inekf_model::R] = init_r;
    }

    MatrixZZ R = build_R(obs.z[obs::DIST], predicted_z_to_v, 2);
    auto status = ekf_.update_forward_gated(
        measure_func, z, R, reset_X, reset_b,
        chi2_threshold, max_reject, q_scale_increase, q_scale_decay
    );

    if (status == aimer::filter::InEKFUpdateStatus::RESET) {
        tracked_armor_id_ = 0;
        debug::print(debug::PrintMode::WARNING, "SpInekfMotion",
            "InEKF reset due to {} consecutive rejections (dual armor)", max_reject);
    }

    b = ekf_.get_b();
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_max");
    double l_abs_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.l_abs_max");
    double h_abs_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.h_abs_max");

    b[sp_inekf_model::R] = std::clamp(b[sp_inekf_model::R], r_min, r_max);
    b[sp_inekf_model::L] = std::clamp(b[sp_inekf_model::L], -l_abs_max, l_abs_max);
    b[sp_inekf_model::H] = std::clamp(b[sp_inekf_model::H], -h_abs_max, h_abs_max);

    if (runtime_param::get_param<bool>("AutoAim.Predictor.SpEKF.force_zero_vz")) {
        b[sp_inekf_model::VZ] = 0;
    }
    ekf_.set_b(b);

    last_update_time_ = timestamp;
}

// ============================================================================
// 预测与查询
// ============================================================================

Eigen::Vector3d SpInekfMotion::predict_center(double dt) const {
    auto X = ekf_.get_X();
    auto b = ekf_.get_b();
    return Eigen::Vector3d(
        X.t.x() + b[sp_inekf_model::VX] * dt,
        X.t.y() + b[sp_inekf_model::VY] * dt,
        b[sp_inekf_model::ZC] + b[sp_inekf_model::VZ] * dt
    );
}

Eigen::Vector3d SpInekfMotion::predict_armor_pos(int armor_idx, double dt) const {
    int rel_idx = ((armor_idx % armor_num_) + armor_num_) % armor_num_;
    int abs_id = (tracked_armor_id_ + rel_idx) % armor_num_;

    auto X = ekf_.get_X();
    auto b = ekf_.get_b();

    double xc = X.t.x() + b[sp_inekf_model::VX] * dt;
    double yc = X.t.y() + b[sp_inekf_model::VY] * dt;
    double zc = b[sp_inekf_model::ZC] + b[sp_inekf_model::VZ] * dt;
    double theta = X.theta + b[sp_inekf_model::OMEGA] * dt;

    double armor_angle = theta + abs_id * (2.0 * M_PI / armor_num_);
    bool use_l_h = (armor_num_ == 4) && (abs_id % 2 == 1);
    double r_actual = use_l_h ? (b[sp_inekf_model::R] + b[sp_inekf_model::L]) : b[sp_inekf_model::R];
    double z_actual = use_l_h ? (zc + b[sp_inekf_model::H]) : zc;

    return Eigen::Vector3d(
        xc + r_actual * std::cos(armor_angle),
        yc + r_actual * std::sin(armor_angle),
        z_actual
    );
}

Eigen::Vector3d SpInekfMotion::get_armor_pos() const {
    return predict_armor_pos(0, 0);
}

Eigen::Vector3d SpInekfMotion::get_velocity() const {
    auto b = ekf_.get_b();
    return Eigen::Vector3d(
        b[sp_inekf_model::VX],
        b[sp_inekf_model::VY],
        b[sp_inekf_model::VZ]
    );
}

double SpInekfMotion::get_theta() const {
    return ekf_.get_X().theta + tracked_armor_id_ * (2.0 * M_PI / armor_num_);
}

double SpInekfMotion::get_omega() const {
    return ekf_.get_b()[sp_inekf_model::OMEGA];
}

double SpInekfMotion::get_radius() const {
    return ekf_.get_b()[sp_inekf_model::R];
}

double SpInekfMotion::get_another_radius() const {
    auto b = ekf_.get_b();
    return b[sp_inekf_model::R] + b[sp_inekf_model::L];
}

double SpInekfMotion::get_dz() const {
    return ekf_.get_b()[sp_inekf_model::H];
}

std::vector<Eigen::Vector3d> SpInekfMotion::compute_all_armors(double dt) const {
    std::vector<Eigen::Vector3d> result;
    result.reserve(armor_num_);
    for (int i = 0; i < armor_num_; ++i) {
        result.push_back(predict_armor_pos(i, dt));
    }
    return result;
}

void SpInekfMotion::log_state(const std::string& prefix) const {
    auto X = ekf_.get_X();
    auto b = ekf_.get_b();

    rr::scalar(prefix + "/xc", X.t.x());
    rr::scalar(prefix + "/yc", X.t.y());
    rr::scalar(prefix + "/zc", b[sp_inekf_model::ZC]);
    rr::scalar(prefix + "/vx", b[sp_inekf_model::VX]);
    rr::scalar(prefix + "/vy", b[sp_inekf_model::VY]);
    rr::scalar(prefix + "/vz", b[sp_inekf_model::VZ]);
    rr::scalar(prefix + "/theta", X.theta * 57.3);
    rr::scalar(prefix + "/omega", b[sp_inekf_model::OMEGA]);
    rr::scalar(prefix + "/r", b[sp_inekf_model::R]);
    rr::scalar(prefix + "/l", b[sp_inekf_model::L]);
    rr::scalar(prefix + "/h", b[sp_inekf_model::H]);
    rr::scalar(prefix + "/tracked_id", tracked_armor_id_);
}

void SpInekfMotion::reset() {
    initialized_ = false;
    tracked_armor_id_ = 0;
    last_detector_id_ = -1;
}

// ============================================================================
// 噪声矩阵
// ============================================================================

SpInekfMotion::MatrixXX SpInekfMotion::build_Q(double dt) const {
    MatrixXX Q = MatrixXX::Zero();

    const double q_acc = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_acc");
    const double q_ang_acc = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_ang_acc");
    const double q_r_rw = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_r_rw");
    const double q_l_rw = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_l_rw");
    const double q_h_rw = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_h_rw");

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double a = dt4 * 0.25;
    const double b = dt3 * 0.5;
    const double c = dt2;

    auto fill_block = [&](int pos_idx, int vel_idx, double var_acc) {
        Q(pos_idx, pos_idx) = a * var_acc;
        Q(pos_idx, vel_idx) = b * var_acc;
        Q(vel_idx, pos_idx) = b * var_acc;
        Q(vel_idx, vel_idx) = c * var_acc;
    };

    // xi_x (index 1) <-> vx (index 4 in total state = 1 in b)
    fill_block(1, 1 + sp_inekf_model::VX, q_acc);
    // xi_y (index 2) <-> vy
    fill_block(2, 1 + sp_inekf_model::VY, q_acc);
    // xi_theta (index 0) <-> omega
    fill_block(0, 1 + sp_inekf_model::OMEGA, q_ang_acc);
    // zc <-> vz
    fill_block(1 + sp_inekf_model::ZC, 1 + sp_inekf_model::VZ, q_acc);

    // 几何参数随机游走
    Q(1 + sp_inekf_model::R, 1 + sp_inekf_model::R) = q_r_rw * dt;
    Q(1 + sp_inekf_model::L, 1 + sp_inekf_model::L) = q_l_rw * dt;
    Q(1 + sp_inekf_model::H, 1 + sp_inekf_model::H) = q_h_rw * dt;

    return Q;
}

SpInekfMotion::MatrixZZ SpInekfMotion::build_R(double distance, double z_to_v, int observed_armor_count) const {
    MatrixZZ R = MatrixZZ::Zero();

    const double r_angle = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_angle");
    const std::string r_dis_model = runtime_param::get_param<std::string>("AutoAim.Predictor.SpEKF.r_dis_model");
    const double r_dis_base = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_dis_base");
    const double r_dis_log_k = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_dis_log_k");
    const double r_dis_quad_k = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_dis_quad_k");
    const double r_dis_pow4_k = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_dis_pow4_k");

    const double r_armor_yaw_base = (observed_armor_count >= 2)
        ? runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_armor_yaw_double")
        : runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_armor_yaw_single");

    const double z_to_v_clamped = std::clamp(std::abs(z_to_v), 0.0, 1.5);
    const double adaptive_factor = std::log(z_to_v_clamped + 1.0) + 1.0;
    const double r_armor_yaw = r_armor_yaw_base * adaptive_factor;
    const double dis_abs = std::max(0.0, distance);

    double r_dis = r_dis_base;
    if (r_dis_model == "quadratic") {
        r_dis += r_dis_quad_k * dis_abs * dis_abs;
    } else if (r_dis_model == "pow4") {
        r_dis += r_dis_pow4_k * std::pow(dis_abs, 4.0);
    } else {
        r_dis += r_dis_log_k * std::log(dis_abs + 1.0);
    }
    r_dis = std::max(r_dis, 1e-6);

    R(sp_inekf_model::YAW, sp_inekf_model::YAW) = r_angle;
    R(sp_inekf_model::PITCH, sp_inekf_model::PITCH) = r_angle;
    R(sp_inekf_model::DIS, sp_inekf_model::DIS) = r_dis;
    R(sp_inekf_model::ARMOR_YAW, sp_inekf_model::ARMOR_YAW) = r_armor_yaw;

    return R;
}

} // namespace autoaim::predictor
