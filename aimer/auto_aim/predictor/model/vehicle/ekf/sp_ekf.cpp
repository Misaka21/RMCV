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
        reset_state[sp_model::XC] = xc;
        reset_state[sp_model::YC] = yc;
        reset_state[sp_model::ZC] = obs.pos.z();
        reset_state[sp_model::THETA] = orient_yaw;  // 假设是 id=0
        reset_state[sp_model::R] = init_r;
    }

    // ⭐ 关键修正: 自适应因子要与 sp_vision 的定义保持一致
    // sp_vision 使用的是 INWARD 装甲板朝向:
    //   delta = armor_yaw_inward - position_yaw
    // 当前状态里 inner_z[ARMOR_YAW] 是 OUTWARD，需要先转回 INWARD 再比较，
    // 否则会出现整体偏移 π，导致自适应因子长期饱和。
    double predicted_armor_yaw_inward = inner_z[sp_model::ARMOR_YAW] - M_PI;
    double predicted_z_to_v = std::abs(aimer::math::angle_diff(
        predicted_armor_yaw_inward, inner_z[sp_model::YAW]));

    // 观测更新
    MatrixZZ R = build_R(obs.z[obs::DIST], predicted_z_to_v);
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

    // 单装甲板直接更新
    if (armors.size() == 1) {
        update(primary, timestamp);
        return;
    }

    // ⭐ 前置过滤: z_to_v 异常值检测
    constexpr double Z_TO_V_MAX = 1.6;
    if (std::abs(primary.z_to_v()) > Z_TO_V_MAX) {
        debug::print(debug::PrintMode::WARNING, "SpMotion",
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
    SpPredict predict_func(dt);
    MatrixXX Q = build_Q(dt);
    ekf_.predict_forward_scaled(predict_func, Q);

    // 2. 匹配装甲板 ID
    int matched_id = match_armor(primary);
    tracked_armor_id_ = matched_id;
    last_detector_id_ = primary.id;  // 记录主装甲板的 detector ID

    // 3. 用主装甲板做观测更新
    const auto& obs = primary.observation;
    double orient_yaw = obs.z[obs::ARMOR_YAW] + M_PI;  // OUTWARD

    SpMeasure measure_func(matched_id, armor_num_);

    VectorZ inner_z;
    VectorX x = ekf_.get_x();
    double x_arr[sp_model::N_X], z_arr[sp_model::N_Z];
    for (int i = 0; i < sp_model::N_X; ++i) x_arr[i] = x[i];
    measure_func(x_arr, z_arr);
    for (int i = 0; i < sp_model::N_Z; ++i) inner_z[i] = z_arr[i];

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
        reset_state[sp_model::XC] = xc;
        reset_state[sp_model::YC] = yc;
        reset_state[sp_model::ZC] = obs.pos.z();
        reset_state[sp_model::THETA] = orient_yaw;
        reset_state[sp_model::R] = init_r;
    }

    // ⭐ 与单板更新保持一致：OUTWARD -> INWARD 后再计算自适应角差
    double predicted_armor_yaw_inward = inner_z[sp_model::ARMOR_YAW] - M_PI;
    double predicted_z_to_v = std::abs(aimer::math::angle_diff(
        predicted_armor_yaw_inward, inner_z[sp_model::YAW]));

    // 双装甲板时观测噪声更小
    MatrixZZ R = build_R(obs.z[obs::DIST], predicted_z_to_v, 2);
    auto status = ekf_.update_forward_gated(
        measure_func, z, R, reset_state,
        chi2_threshold, max_reject, q_scale_increase, q_scale_decay
    );

    if (status == aimer::filter::UpdateStatus::RESET) {
        tracked_armor_id_ = 0;
        debug::print(debug::PrintMode::WARNING, "SpMotion",
            "EKF reset due to {} consecutive rejections (dual armor)", max_reject);
    }

    // 后处理
    x = ekf_.get_x();
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

    double q_pos = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_pos");
    double q_vel = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_vel");
    double q_theta = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_theta");
    double q_omega = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_omega");
    double q_r = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_r");
    double q_l = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_l");
    double q_h = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.q_h");

    // 位置-速度块
    Q(sp_model::XC, sp_model::XC) = q_pos * dt;
    Q(sp_model::VX, sp_model::VX) = q_vel * dt;
    Q(sp_model::YC, sp_model::YC) = q_pos * dt;
    Q(sp_model::VY, sp_model::VY) = q_vel * dt;
    Q(sp_model::ZC, sp_model::ZC) = q_pos * dt;
    Q(sp_model::VZ, sp_model::VZ) = q_vel * dt;

    // 朝向角-角速度块
    Q(sp_model::THETA, sp_model::THETA) = q_theta * dt;
    Q(sp_model::OMEGA, sp_model::OMEGA) = q_omega * dt;

    // 几何参数 (不乘 dt)
    Q(sp_model::R, sp_model::R) = q_r;
    Q(sp_model::L, sp_model::L) = q_l;
    Q(sp_model::H, sp_model::H) = q_h;

    return Q;
}

SpMotion::MatrixZZ SpMotion::build_R(double distance, double z_to_v, int observed_armor_count) const {
    MatrixZZ R = MatrixZZ::Zero();

    double r_angle = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_angle");
    double r_dis_k = runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_dis_k");

    // 基础朝向角噪声
    double r_armor_yaw_base = (observed_armor_count >= 2)
        ? runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_armor_yaw_double")
        : runtime_param::get_param<double>("AutoAim.Predictor.SpEKF.r_armor_yaw_single");

    // ⭐ 自适应噪声 (来自 sp_vision_25)
    // 重要: z_to_v 应该是 EKF 预测的角度，不是观测值！
    //
    // 原理: 侧对时 PnP 朝向角精度下降，需要增大 R
    // 关键: 用预测值计算，这样:
    //   - 正常观测: 预测和观测都接近，R 适中，正常更新
    //   - 离群观测: 预测值正常，R 正常，innovation 大，被 Gating 拒绝
    //
    // 如果用观测值计算 R (错误做法):
    //   - 离群观测 z_to_v 异常大 → R 变大 → 马氏距离变小 → 反而通过 Gating！

    // 防止异常值: z_to_v 理论范围 [0, π/2]，clamp 到 [0, 1.5]
    double z_to_v_clamped = std::clamp(std::abs(z_to_v), 0.0, 1.5);

    // 使用 log 函数平滑过渡
    double adaptive_factor = std::log(z_to_v_clamped + 1.0) + 1.0;
    // z_to_v = 0.0 → factor = 1.0 (正对)
    // z_to_v = 0.5 → factor ≈ 1.4
    // z_to_v = 1.0 → factor ≈ 1.7 (侧对)
    // z_to_v = 1.5 → factor ≈ 1.9 (max)

    double r_armor_yaw = r_armor_yaw_base * adaptive_factor;

    R(sp_model::YAW, sp_model::YAW) = r_angle;
    R(sp_model::PITCH, sp_model::PITCH) = r_angle;
    R(sp_model::DIS, sp_model::DIS) = r_dis_k * std::pow(distance, 4.0);
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
