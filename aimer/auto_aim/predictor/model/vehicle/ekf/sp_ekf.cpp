/**
 * @file sp_motion.cpp
 * @brief SP 整车旋转模型实现 - 移植自 sp_vision_25
 */

#include "sp_ekf.hpp"

#include <cmath>
#include <algorithm>

#include "aimer/common/math/math.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/plotter/plotter.hpp"

namespace autoaim::predictor {

// ============================================================================
// SpMotion 实现
// ============================================================================

SpMotion::SpMotion(int armor_num) : armor_num_(armor_num) {}

void SpMotion::init(const ArmorData& armor, double timestamp) {
    const auto& obs = armor.observation;

    // 从观测提取
    double xa = obs.pos.x();
    double ya = obs.pos.y();
    double za = obs.pos.z();

    // 装甲板朝向 (PnP 输出的是装甲板面朝方向，即 INWARD)
    // 需要转换为 OUTWARD: 从中心指向装甲板
    double armor_yaw_inward = obs.z[obs::ARMOR_YAW];
    double armor_theta = armor_yaw_inward + M_PI;  // OUTWARD

    // 初始半径
    double r = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.init_r");

    // 从装甲板位置反推旋转中心
    // OUTWARD: center = armor - r * (cos θ, sin θ)
    double xc = xa - r * std::cos(armor_theta);
    double yc = ya - r * std::sin(armor_theta);
    double zc = za;  // 初始时 h = 0

    // 初始化状态 (11维)
    // 假设初始追踪的是 id=0 装甲板
    // theta = armor_theta (因为 armor_angle = theta + 0 * 2π/N = theta)
    VectorX x0 = VectorX::Zero();
    x0[sp_model::XC] = xc;
    x0[sp_model::YC] = yc;
    x0[sp_model::ZC] = zc;
    x0[sp_model::THETA] = armor_theta;  // OUTWARD, 指向 id=0 装甲板
    x0[sp_model::R] = r;
    x0[sp_model::L] = 0;  // 初始半径差为 0
    x0[sp_model::H] = 0;  // 初始高度差为 0
    // 速度初始为 0

    ekf_.init(x0);

    tracked_armor_id_ = 0;  // 假设初始追踪的是 state_id=0
    last_detector_id_ = armor.id;  // 记录初始的 detector ID
    last_update_time_ = timestamp;
    initialized_ = true;

    debug::print(debug::PrintMode::DEBUG, "SpMotion",
        "Init: armor.id={}, theta={:.1f}°, r={:.3f}, center=({:.2f},{:.2f},{:.2f})",
        armor.id, armor_theta * 180.0 / M_PI, r, xc, yc, zc);
}

int SpMotion::match_armor(const ArmorData& armor) const {
    // ⭐ 利用 armor.id 的稳定性：
    // 如果是同一个物理装甲板（detector ID 相同），帧间 state_id 不会变
    // 帧间时间 ~5ms，即使高速陀螺 500°/s 也只旋转 2.5°，不会跨越 90° 边界
    if (armor.id == last_detector_id_ && armor.id >= 0) {
        return tracked_armor_id_;  // 保持 state_id 稳定
    }

    // 新装甲板或 ID 变化，重新计算最佳匹配
    const auto& obs = armor.observation;

    // 观测的装甲板位置和朝向
    double obs_yaw = obs.z[obs::YAW];
    double obs_armor_yaw = obs.z[obs::ARMOR_YAW] + M_PI;  // 转为 OUTWARD

    VectorX x = ekf_.get_x();

    // 1. 计算所有预测装甲板位置和距离
    struct Candidate {
        int id;
        double distance;
        double angle_error;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(armor_num_);

    for (int i = 0; i < armor_num_; ++i) {
        Eigen::Vector3d pred_pos = h_armor_xyz(x, i);
        double dist = (pred_pos - obs.pos).norm();

        // 预测的装甲板朝向 (OUTWARD)
        double pred_armor_yaw = x[sp_model::THETA] + i * (2.0 * M_PI / armor_num_);
        // 预测的方位角
        double pred_yaw = std::atan2(pred_pos.y(), pred_pos.x());

        // 角度误差
        double yaw_diff = std::abs(aimer::math::angle_diff(obs_yaw, pred_yaw));
        double orient_diff = std::abs(aimer::math::angle_diff(obs_armor_yaw, pred_armor_yaw));
        double angle_error = yaw_diff + orient_diff;

        candidates.push_back({i, dist, angle_error});
    }

    // 2. 按距离排序，取最近3个
    std::sort(candidates.begin(), candidates.end(),
        [](const auto& a, const auto& b) { return a.distance < b.distance; });

    int num_to_check = std::min(3, static_cast<int>(candidates.size()));

    // 3. 在最近的几个中找角度误差最小的
    int best_id = candidates[0].id;
    double min_error = candidates[0].angle_error;

    for (int i = 1; i < num_to_check; ++i) {
        if (candidates[i].angle_error < min_error) {
            min_error = candidates[i].angle_error;
            best_id = candidates[i].id;
        }
    }

    return best_id;
}

Eigen::Vector3d SpMotion::h_armor_xyz(const VectorX& x, int id) const {
    bool use_l_h = (armor_num_ == 4) && (id % 2 == 1);
    double r_actual = use_l_h ? (x[sp_model::R] + x[sp_model::L]) : x[sp_model::R];
    double z_actual = use_l_h ? (x[sp_model::ZC] + x[sp_model::H]) : x[sp_model::ZC];
    double angle = x[sp_model::THETA] + id * 2.0 * M_PI / armor_num_;

    return Eigen::Vector3d(
        x[sp_model::XC] + r_actual * std::cos(angle),
        x[sp_model::YC] + r_actual * std::sin(angle),
        z_actual
    );
}

void SpMotion::update(const ArmorData& armor, double timestamp) {
    if (!initialized_) {
        init(armor, timestamp);
        return;
    }

    const auto& obs = armor.observation;

    // ⭐ 前置过滤: z_to_v 异常值检测
    // z_to_v 理论范围 [0, π/2] ≈ [0, 1.57]，超出说明 PnP 有问题
    constexpr double Z_TO_V_MAX = 1.6;  // 略大于 π/2
    if (std::abs(armor.z_to_v()) > Z_TO_V_MAX) {
        debug::print(debug::PrintMode::WARNING, "SpMotion",
            "Reject observation: z_to_v={:.3f} out of range", armor.z_to_v());
        return;  // 直接拒绝，不更新 EKF
    }

    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    // 1. 预测
    SpPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward_scaled(predict_func, Q);

    // 2. 匹配装甲板 ID
    int matched_id = match_armor(armor);
    tracked_armor_id_ = matched_id;
    last_detector_id_ = armor.id;  // 记录这一帧的 detector ID

    // 3. 观测更新 (使用匹配的 armor_id)
    double orient_yaw = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD

    // 使用带 armor_id 的观测函数
    SpMeasure measure_func(matched_id, armor_num_);

    // 获取 EKF 内部预测的观测值 (用于连续化)
    VectorZ inner_z;
    VectorX x = ekf_.get_x();
    double x_arr[sp_model::N_X], z_arr[sp_model::N_Z];
    for (int i = 0; i < sp_model::N_X; ++i) x_arr[i] = x[i];
    measure_func(x_arr, z_arr);
    for (int i = 0; i < sp_model::N_Z; ++i) inner_z[i] = z_arr[i];

    // 连续化
    VectorZ z;
    z[sp_model::YAW] = aimer::math::get_closest_angle(obs.z[obs::YAW], inner_z[sp_model::YAW]);
    z[sp_model::PITCH] = obs.z[obs::PITCH];
    z[sp_model::DIS] = obs.z[obs::DIST];
    z[sp_model::ARMOR_YAW] = aimer::math::get_closest_angle(orient_yaw, inner_z[sp_model::ARMOR_YAW]);

    // 读取门限参数
    double chi2_threshold = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.chi2_threshold");
    int64_t max_reject = runtime_param::get_param<int64_t>("AutoAim.Predictor.SpEKF.Gating.max_reject");
    double q_scale_increase = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.q_scale_increase");
    double q_scale_decay = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.q_scale_decay");

    // 构建重置状态
    VectorX reset_state = VectorX::Zero();
    {
        double init_r = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.init_r");
        double xc = obs.pos.x() - init_r * std::cos(orient_yaw);
        double yc = obs.pos.y() - init_r * std::sin(orient_yaw);
        // reset_state 的 THETA 始终对应 state_id=0 的朝向
        double theta_state0 = orient_yaw - matched_id * (2.0 * M_PI / armor_num_);
        reset_state[sp_model::XC] = xc;
        reset_state[sp_model::YC] = yc;
        reset_state[sp_model::ZC] = obs.pos.z();
        reset_state[sp_model::THETA] = aimer::math::reduced_angle(theta_state0);
        reset_state[sp_model::R] = init_r;
    }

    // 与 sp_vision 对齐: delta 使用观测几何（|z_to_v|），而非预测值
    MatrixZZ R = build_R(obs.z[obs::DIST], std::abs(armor.z_to_v()));
    auto status = ekf_.update_forward_gated(
        measure_func, z, R, reset_state,
        chi2_threshold, max_reject, q_scale_increase, q_scale_decay
    );

    if (status == aimer::filter::UpdateStatus::RESET) {
        tracked_armor_id_ = 0;
        debug::print(debug::PrintMode::WARNING, "SpMotion",
            "EKF reset due to {} consecutive rejections", max_reject);
    }

    // 后处理：限制参数范围
    x = ekf_.get_x();
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_max");
    double l_abs_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.l_abs_max");
    double h_abs_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.h_abs_max");

    x[sp_model::R] = std::clamp(x[sp_model::R], r_min, r_max);
    x[sp_model::L] = std::clamp(x[sp_model::L], -l_abs_max, l_abs_max);
    x[sp_model::H] = std::clamp(x[sp_model::H], -h_abs_max, h_abs_max);

    // 强制 Z 轴速度为 0
    if (runtime_param::get_param<bool>("AutoAim.Predictor.SpEKF.force_zero_vz")) {
        x[sp_model::VZ] = 0;
    }
    ekf_.set_x(x);

    last_update_time_ = timestamp;
}

void SpMotion::update(const std::vector<ArmorData>& armors, double timestamp) {
    if (armors.empty()) return;

    // ⭐ 追踪目标选择
    // 规则：如果已追踪的面积 >= keep_ratio * max_area，继续追踪；否则切换到最大的
    const auto& primary = [&]() -> const ArmorData& {
        if (armors.size() == 1) {
            return armors[0];
        }

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

        if (tracked_idx >= 0 && tracked_area >= keep_ratio * max_area) {
            return armors[tracked_idx];
        }
        return armors[max_area_idx];
    }();

    // 更新顺序: 主装甲板优先，其余装甲板同帧继续更新（与 sp_vision 同帧多观测行为对齐）
    std::vector<const ArmorData*> ordered_armors;
    ordered_armors.reserve(armors.size());
    ordered_armors.push_back(&primary);
    for (const auto& armor : armors) {
        if (&armor != &primary) ordered_armors.push_back(&armor);
    }

    constexpr double Z_TO_V_MAX = 1.6;
    auto obs_valid = [&](const ArmorData& armor) {
        return std::abs(armor.z_to_v()) <= Z_TO_V_MAX;
    };

    if (!initialized_) {
        for (const ArmorData* armor : ordered_armors) {
            if (obs_valid(*armor)) {
                init(*armor, timestamp);
                return;
            }
        }

        debug::print(debug::PrintMode::WARNING, "SpMotion",
            "Reject all observations in init: z_to_v out of range");
        return;
    }

    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    const ArmorData* frame_primary = &primary;
    if (!obs_valid(*frame_primary)) {
        for (const ArmorData* armor : ordered_armors) {
            if (obs_valid(*armor)) {
                frame_primary = armor;
                break;
            }
        }
        if (frame_primary != &primary) {
            debug::print(debug::PrintMode::WARNING, "SpMotion",
                "Primary z_to_v invalid ({:.3f}), fallback to armor.id={}",
                primary.z_to_v(), frame_primary->id);
        }
    }

    // 1. 预测
    SpPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward_scaled(predict_func, Q);

    // 2. 读取门限参数
    double chi2_threshold = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.chi2_threshold");
    int64_t max_reject = runtime_param::get_param<int64_t>("AutoAim.Predictor.SpEKF.Gating.max_reject");
    double q_scale_increase = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.q_scale_increase");
    double q_scale_decay = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.Gating.q_scale_decay");
    // 同帧次观测保护: 防止错配观测拉崩滤波器
    double secondary_pos_gate = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.multi_update_pos_gate");
    double secondary_orient_gate = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.multi_update_orient_gate");
    double secondary_r_scale = std::max(
        1.0, runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.secondary_r_scale"));
    double secondary_nis_scale = std::max(
        1.0, runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.secondary_nis_scale"));
    double multi_update_min_omega = std::max(
        0.0, runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.multi_update_min_omega"));
    bool allow_secondary_update = std::abs(ekf_.get_x()[sp_model::OMEGA]) >= multi_update_min_omega;

    // 3. 同帧多观测更新（主板优先，随后其余板）
    for (const ArmorData* armor_ptr : ordered_armors) {
        const ArmorData& armor = *armor_ptr;

        if (!obs_valid(armor)) {
            debug::print(debug::PrintMode::WARNING, "SpMotion",
                "Skip observation in multi-update: z_to_v={:.3f} out of range", armor.z_to_v());
            continue;
        }

        const auto& obs = armor.observation;
        int matched_id = match_armor(armor);
        bool is_primary = (armor_ptr == frame_primary);

        if (is_primary) {
            tracked_armor_id_ = matched_id;
            last_detector_id_ = armor.id;
        }

        double orient_yaw = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD
        SpMeasure measure_func(matched_id, armor_num_);

        VectorX x = ekf_.get_x();
        if (!is_primary) {
            if (!allow_secondary_update) {
                continue;
            }

            Eigen::Vector3d pred_pos = h_armor_xyz(x, matched_id);
            double pos_err = (pred_pos - obs.pos).norm();
            double pred_orient_yaw = x[sp_model::THETA] + matched_id * (2.0 * M_PI / armor_num_);
            double orient_err = std::abs(aimer::math::angle_diff(orient_yaw, pred_orient_yaw));
            if (pos_err > secondary_pos_gate || orient_err > secondary_orient_gate) {
                debug::print(debug::PrintMode::WARNING, "SpMotion",
                    "Skip secondary update: armor.id={}, pos_err={:.3f}m, orient_err={:.1f}deg",
                    armor.id, pos_err, orient_err * 180.0 / M_PI);
                continue;
            }
        }

        VectorZ inner_z;
        double x_arr[sp_model::N_X], z_arr[sp_model::N_Z];
        for (int i = 0; i < sp_model::N_X; ++i) x_arr[i] = x[i];
        measure_func(x_arr, z_arr);
        for (int i = 0; i < sp_model::N_Z; ++i) inner_z[i] = z_arr[i];

        VectorZ z;
        z[sp_model::YAW] = aimer::math::get_closest_angle(obs.z[obs::YAW], inner_z[sp_model::YAW]);
        z[sp_model::PITCH] = obs.z[obs::PITCH];
        z[sp_model::DIS] = obs.z[obs::DIST];
        z[sp_model::ARMOR_YAW] = aimer::math::get_closest_angle(orient_yaw, inner_z[sp_model::ARMOR_YAW]);

        VectorX reset_state = VectorX::Zero();
        {
            double init_r = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.init_r");
            double xc = obs.pos.x() - init_r * std::cos(orient_yaw);
            double yc = obs.pos.y() - init_r * std::sin(orient_yaw);
            double theta_state0 = orient_yaw - matched_id * (2.0 * M_PI / armor_num_);
            reset_state[sp_model::XC] = xc;
            reset_state[sp_model::YC] = yc;
            reset_state[sp_model::ZC] = obs.pos.z();
            reset_state[sp_model::THETA] = aimer::math::reduced_angle(theta_state0);
            reset_state[sp_model::R] = init_r;
        }

        MatrixZZ R = build_R(obs.z[obs::DIST], std::abs(armor.z_to_v()));
        if (!is_primary) {
            R *= secondary_r_scale;
        }
        if (!is_primary) {
            // 次观测不参与 reject_count/reset，避免启停时被多观测放大导致追踪恶化
            double nis = ekf_.compute_mahalanobis_sq(measure_func, z, R);
            double threshold = chi2_threshold;
            if (ekf_.get_q_scale() > 1.0) threshold *= ekf_.get_q_scale();
            threshold *= secondary_nis_scale;
            if (nis > threshold) {
                debug::print(debug::PrintMode::WARNING, "SpMotion",
                    "Skip secondary update by NIS: armor.id={}, nis={:.2f}, th={:.2f}",
                    armor.id, nis, threshold);
                continue;
            }
            ekf_.update_forward(measure_func, z, R);
        } else {
            auto status = ekf_.update_forward_gated(
                measure_func, z, R, reset_state,
                chi2_threshold, max_reject, q_scale_increase, q_scale_decay
            );

            if (status == aimer::filter::UpdateStatus::RESET) {
                tracked_armor_id_ = 0;
                last_detector_id_ = armor.id;
                debug::print(debug::PrintMode::WARNING, "SpMotion",
                    "EKF reset due to {} consecutive rejections (matched_id={})",
                    max_reject, matched_id);
            }
        }
    }

    // 4. 后处理
    VectorX x = ekf_.get_x();
    double r_min = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_min");
    double r_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_max");
    double l_abs_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.l_abs_max");
    double h_abs_max = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.h_abs_max");

    x[sp_model::R] = std::clamp(x[sp_model::R], r_min, r_max);
    x[sp_model::L] = std::clamp(x[sp_model::L], -l_abs_max, l_abs_max);
    x[sp_model::H] = std::clamp(x[sp_model::H], -h_abs_max, h_abs_max);

    if (runtime_param::get_param<bool>("AutoAim.Predictor.SpEKF.force_zero_vz")) {
        x[sp_model::VZ] = 0;
    }
    ekf_.set_x(x);

    last_update_time_ = timestamp;
}

SpMotion::MatrixXX SpMotion::build_Q(double dt) const {
    MatrixXX Q = MatrixXX::Zero();

    // Piecewise White Noise Model (sp_vision_25 同款)
    // 参数含义: 加速度方差，Q 矩阵通过 dt⁴/4, dt³/2, dt² 结构自动耦合位置-速度
    double v_pos = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_acc_pos");
    double v_yaw = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_acc_yaw");

    double q_r = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_r");
    double q_l = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_l");
    double q_h = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_h");

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt2 * dt2;
    double a = dt4 * 0.25;
    double b = dt3 * 0.5;
    double c = dt2;

    // x/vx, y/vy, z/vz 3个分块
    Q(sp_model::XC, sp_model::XC) = a * v_pos;
    Q(sp_model::XC, sp_model::VX) = b * v_pos;
    Q(sp_model::VX, sp_model::XC) = b * v_pos;
    Q(sp_model::VX, sp_model::VX) = c * v_pos;

    Q(sp_model::YC, sp_model::YC) = a * v_pos;
    Q(sp_model::YC, sp_model::VY) = b * v_pos;
    Q(sp_model::VY, sp_model::YC) = b * v_pos;
    Q(sp_model::VY, sp_model::VY) = c * v_pos;

    Q(sp_model::ZC, sp_model::ZC) = a * v_pos;
    Q(sp_model::ZC, sp_model::VZ) = b * v_pos;
    Q(sp_model::VZ, sp_model::ZC) = b * v_pos;
    Q(sp_model::VZ, sp_model::VZ) = c * v_pos;

    // θ/ω 分块
    Q(sp_model::THETA, sp_model::THETA) = a * v_yaw;
    Q(sp_model::THETA, sp_model::OMEGA) = b * v_yaw;
    Q(sp_model::OMEGA, sp_model::THETA) = b * v_yaw;
    Q(sp_model::OMEGA, sp_model::OMEGA) = c * v_yaw;

    // 几何参数默认与 sp_vision 一致保持常量；保留极小噪声可选
    Q(sp_model::R, sp_model::R) = q_r;
    Q(sp_model::L, sp_model::L) = q_l;
    Q(sp_model::H, sp_model::H) = q_h;

    return Q;
}

SpMotion::MatrixZZ SpMotion::build_R(double distance, double z_to_v) const {
    MatrixZZ R = MatrixZZ::Zero();

    // sp_vision_25 同款 R 构造:
    //   R_yaw = R_pitch = r_yaw_pitch            (固定)
    //   R_dis  = r_dis_base × (log(|delta|+1)+1)  (delta 越大=越侧对，距离噪声越大)
    //   R_orient = r_base + r_scale × log(dis+1)   (距离越远，朝向角噪声越大)
    // 注意: R_dis 依赖的是朝向角 delta 而非距离!
    double r_yaw_pitch = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_yaw_pitch");
    double r_dis_base = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_dis_base");
    double r_armor_yaw_base = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_armor_yaw_base");
    double r_armor_yaw_dis_scale = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_armor_yaw_dis_scale");

    double delta = std::abs(z_to_v);
    double dis = std::abs(distance);

    double r_dis = r_dis_base * (std::log(delta + 1.0) + 1.0);
    double r_armor_yaw = r_armor_yaw_base + r_armor_yaw_dis_scale * std::log(dis + 1.0);

    R(sp_model::YAW, sp_model::YAW) = r_yaw_pitch;
    R(sp_model::PITCH, sp_model::PITCH) = r_yaw_pitch;
    R(sp_model::DIS, sp_model::DIS) = r_dis;
    R(sp_model::ARMOR_YAW, sp_model::ARMOR_YAW) = r_armor_yaw;

    return R;
}

Eigen::Vector3d SpMotion::predict_center(double dt) const {
    VectorX x = ekf_.get_x();
    return Eigen::Vector3d(
        x[sp_model::XC] + x[sp_model::VX] * dt,
        x[sp_model::YC] + x[sp_model::VY] * dt,
        x[sp_model::ZC] + x[sp_model::VZ] * dt
    );
}

Eigen::Vector3d SpMotion::predict_armor_pos(int armor_idx, double dt) const {
    // MotionInterface 约定: armor_idx=0 表示当前追踪装甲板。
    // Sp 状态内部用的是绝对 state_id(0..N-1)，这里做一次相对->绝对映射。
    int rel_idx = ((armor_idx % armor_num_) + armor_num_) % armor_num_;
    int abs_id = (tracked_armor_id_ + rel_idx) % armor_num_;

    VectorX x = ekf_.get_x();

    // 预测中心
    double xc = x[sp_model::XC] + x[sp_model::VX] * dt;
    double yc = x[sp_model::YC] + x[sp_model::VY] * dt;
    double zc = x[sp_model::ZC] + x[sp_model::VZ] * dt;

    // 预测朝向
    double theta = x[sp_model::THETA] + x[sp_model::OMEGA] * dt;

    // 装甲板角度
    double armor_angle = theta + abs_id * (2.0 * M_PI / armor_num_);

    // 选择半径和高度
    bool use_l_h = (armor_num_ == 4) && (abs_id % 2 == 1);
    double r_actual = use_l_h ? (x[sp_model::R] + x[sp_model::L]) : x[sp_model::R];
    double z_actual = use_l_h ? (zc + x[sp_model::H]) : zc;

    return Eigen::Vector3d(
        xc + r_actual * std::cos(armor_angle),
        yc + r_actual * std::sin(armor_angle),
        z_actual
    );
}

Eigen::Vector3d SpMotion::get_armor_pos() const {
    // MotionInterface 约定: idx=0 为当前追踪装甲板
    return predict_armor_pos(0, 0);
}

Eigen::Vector3d SpMotion::get_velocity() const {
    VectorX x = ekf_.get_x();
    return Eigen::Vector3d(x[sp_model::VX], x[sp_model::VY], x[sp_model::VZ]);
}

double SpMotion::get_theta() const {
    // 对齐 Spin/Lmtd 的外部语义: 返回“当前追踪装甲板”的 OUTWARD 朝向。
    // 内部状态 x[THETA] 存的是绝对 state_id=0 的朝向。
    VectorX x = ekf_.get_x();
    double theta = x[sp_model::THETA] + tracked_armor_id_ * (2.0 * M_PI / armor_num_);
    return aimer::math::reduced_angle(theta);
}

std::vector<Eigen::Vector3d> SpMotion::compute_all_armors_from_observation(
    const Eigen::Vector3d& /* observed_pos */,
    double /* observed_theta */) const {
    // 直接调用新接口
    return compute_all_armors(0);
}

std::vector<Eigen::Vector3d> SpMotion::compute_all_armors(double dt) const {
    std::vector<Eigen::Vector3d> result;
    result.reserve(armor_num_);

    for (int i = 0; i < armor_num_; ++i) {
        result.push_back(predict_armor_pos(i, dt));
    }

    return result;
}

void SpMotion::output_to_plotter(const std::string& prefix) const {
    VectorX x = ekf_.get_x();

    plotter::add(prefix + "/xc", x[sp_model::XC]);
    plotter::add(prefix + "/yc", x[sp_model::YC]);
    plotter::add(prefix + "/zc", x[sp_model::ZC]);
    plotter::add(prefix + "/vx", x[sp_model::VX]);
    plotter::add(prefix + "/vy", x[sp_model::VY]);
    plotter::add(prefix + "/vz", x[sp_model::VZ]);
    plotter::add(prefix + "/theta", x[sp_model::THETA] * 57.3);  // 转换为度
    plotter::add(prefix + "/omega", x[sp_model::OMEGA]);
    plotter::add(prefix + "/r", x[sp_model::R]);
    plotter::add(prefix + "/l", x[sp_model::L]);
    plotter::add(prefix + "/h", x[sp_model::H]);
    plotter::add(prefix + "/tracked_id", tracked_armor_id_);
}

void SpMotion::reset() {
    initialized_ = false;
    tracked_armor_id_ = 0;
    last_detector_id_ = -1;  // 重置 detector ID
}

}  // namespace autoaim::predictor
