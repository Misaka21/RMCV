// Energy rune predictor implementation (2026)

#include "buff_predictor.hpp"

#include <algorithm>
#include <cmath>

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>

#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"

namespace autobuff::predictor {

namespace {

constexpr double kOmegaConst = M_PI / 3.0;  // pi/3 rad/s

struct BigPhiResidual {
    BigPhiResidual(double t, double y, int dir) : t_(t), y_(y), dir_(dir) {}

    template <typename T>
    bool operator()(const T* const p, T* residual) const {
        // p = [a, w, t_shift, c]
        const T a = p[0];
        const T w = p[1];
        const T t_shift = p[2];
        const T c = p[3];

        const T b = T(2.090) - a;
        const T t = T(t_);

        const T pred = T(dir_) * (-(a / w) * ceres::cos(w * (t + t_shift)) + b * t) + c;
        residual[0] = pred - T(y_);
        return true;
    }

    double t_;
    double y_;
    int dir_;
};

inline std::vector<int> sorted_by_confidence_lit(
    const autobuff::BuffDetectionResult& det)
{
    std::vector<int> ids;
    ids.reserve(NUM_SLOTS);
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!det.targets[i].valid) continue;
        if (!det.targets[i].is_lit) continue;
        ids.push_back(i);
    }
    std::sort(ids.begin(), ids.end(), [&](int a, int b) {
        return det.targets[a].confidence > det.targets[b].confidence;
    });
    return ids;
}

inline int first_valid_slot(const autobuff::BuffDetectionResult& det) {
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (det.targets[i].valid) return i;
    }
    return -1;
}

}  // namespace

// EKF 的 Predict/Measure 仿函数 (需要 template operator(), 不能放在 local class)
struct ConstantEkfPredictFunc {
    double dt;
    template <typename T>
    void operator()(const T x_in[2], T x_out[2]) const {
        x_out[0] = x_in[0] + T(dt) * x_in[1];
        x_out[1] = x_in[1];
    }
};

struct ConstantEkfMeasureFunc {
    template <typename T>
    void operator()(const T x[2], T y[1]) const {
        y[0] = x[0];
    }
};

int BuffPredictor::sgn(double x) {
    return (x > 0) - (x < 0);
}

double BuffPredictor::reduced_angle(double x) {
    return std::atan2(std::sin(x), std::cos(x));
}

double BuffPredictor::closest_angle(double target, double current) {
    double diff = target - current;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return current + diff;
}

void BuffPredictor::update_direction_vote(double phi_meas, double dt) {
    if (!has_last_meas_ || dt < 1e-3) return;

    double dphi = reduced_angle(phi_meas - last_phi_meas_);
    if (std::abs(dphi) < 1e-4) return;

    int v = sgn(dphi);
    dir_votes_ += v;
    dir_votes_ = std::clamp(dir_votes_, -20, 20);

    if (std::abs(dir_votes_) >= 8) {
        dir_ = sgn(static_cast<double>(dir_votes_));
    }
}

void BuffPredictor::update_constant_ekf(double phi_meas, double t, double omega_guess) {
    using Ekf = aimer::filter::AdaptiveEkf<2, 1>;
    using MatrixXX = Ekf::MatrixXX;
    using MatrixYY = Ekf::MatrixYY;
    using VectorX = Ekf::VectorX;
    using VectorY = Ekf::VectorY;

    if (!ekf_inited_) {
        VectorX x0;
        x0 << phi_meas, omega_guess;
        ekf_.init(x0);
        ekf_inited_ = true;
        return;
    }

    double dt = t - last_timestamp_;
    if (dt < 1e-4 || dt > 0.2) {
        // 时间戳异常，直接重置
        VectorX x0;
        x0 << phi_meas, omega_guess;
        ekf_.init(x0);
        return;
    }

    // Predict
    MatrixXX Q = MatrixXX::Zero();
    Q(0, 0) = 2e-4;  // phi process noise
    Q(1, 1) = 5e-3;  // omega process noise (small)
    ekf_.predict_forward_scaled(ConstantEkfPredictFunc{dt}, Q);

    // Measure (unwrap)
    VectorX x_pred = ekf_.get_x();
    double phi_adj = closest_angle(phi_meas, x_pred[0]);

    VectorY y;
    y << phi_adj;
    MatrixYY R = MatrixYY::Identity() * 4e-3;  // measurement noise

    ekf_.update_forward(ConstantEkfMeasureFunc{}, y, R);
}

void BuffPredictor::maybe_reset_big_fit(bool big_active_now, double timestamp, double phi_meas) {
    if (big_active_now && !big_active_) {
        big_active_ = true;
        big_start_time_ = timestamp;
        big_samples_.clear();
        big_model_ = BigSineModel{};
        big_model_.valid = false;

        big_phi_unwrapped_ = phi_meas;
        big_last_phi_ = phi_meas;
        big_has_last_phi_ = true;
    } else if (!big_active_now && big_active_) {
        big_active_ = false;
        big_samples_.clear();
        big_model_.valid = false;
        big_has_last_phi_ = false;
    }
}

void BuffPredictor::push_big_sample(double timestamp, double phi_meas) {
    if (!big_active_) return;
    if (!big_has_last_phi_) {
        big_phi_unwrapped_ = phi_meas;
        big_last_phi_ = phi_meas;
        big_has_last_phi_ = true;
    } else {
        big_phi_unwrapped_ += reduced_angle(phi_meas - big_last_phi_);
        big_last_phi_ = phi_meas;
    }

    double t_rel = timestamp - big_start_time_;
    big_samples_.emplace_back(t_rel, big_phi_unwrapped_);

    // keep window
    constexpr size_t MAX_SAMPLES = 250;
    while (big_samples_.size() > MAX_SAMPLES) {
        big_samples_.pop_front();
    }
}

void BuffPredictor::maybe_solve_big_fit() {
    if (!big_active_) return;
    if (big_samples_.size() < 30) return;

    double t_span = big_samples_.back().first - big_samples_.front().first;
    if (t_span < 0.6) return;

    int dir_use = (dir_ != 0) ? dir_ : 1;

    double p[4] = {
        big_model_.valid ? big_model_.a : 0.90,
        big_model_.valid ? big_model_.w : 1.94,
        big_model_.valid ? big_model_.t_shift : 0.0,
        big_model_.valid ? big_model_.c : big_samples_.front().second
    };

    ceres::Problem problem;
    for (const auto& [t, y] : big_samples_) {
        auto* cost = new ceres::AutoDiffCostFunction<BigPhiResidual, 1, 4>(
            new BigPhiResidual(t, y, dir_use));
        problem.AddResidualBlock(cost, new ceres::HuberLoss(0.1), p);
    }

    problem.SetParameterLowerBound(p, 0, 0.780);
    problem.SetParameterUpperBound(p, 0, 1.045);
    problem.SetParameterLowerBound(p, 1, 1.884);
    problem.SetParameterUpperBound(p, 1, 2.000);
    problem.SetParameterLowerBound(p, 2, -0.5);
    problem.SetParameterUpperBound(p, 2, 0.5);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 40;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        big_model_.valid = false;
        return;
    }

    big_model_.valid = true;
    big_model_.dir = dir_use;
    big_model_.start_time = big_start_time_;
    big_model_.a = p[0];
    big_model_.w = p[1];
    big_model_.t_shift = p[2];
    big_model_.c = p[3];
}

BuffSnapshot BuffPredictor::predict(const BuffDetectionResult& det) {
    BuffSnapshot snap;
    snap.frame_id = det.frame_id;
    snap.timestamp = det.timestamp;
    snap.self_state = det.robot_state;

    // 选择 lit slots (按规则期待数量做 clamp)
    int want_lit = 0;
    if (det.robot_state.aim_mode == aimer::AimMode::ENERGY_SMALL) want_lit = 1;
    if (det.robot_state.aim_mode == aimer::AimMode::ENERGY_LARGE) want_lit = 2;

    auto lit_ids = sorted_by_confidence_lit(det);
    if (want_lit > 0 && static_cast<int>(lit_ids.size()) > want_lit) {
        lit_ids.resize(want_lit);
    }
    for (int id : lit_ids) {
        snap.lit_mask |= static_cast<uint8_t>(1u << id);
    }
    snap.lit_count = static_cast<int>(lit_ids.size());
    snap.recommended_slot = lit_ids.empty() ? -1 : lit_ids.front();

    // 跟踪角度测量 (用于方向/拟合)
    const int track_slot = (snap.recommended_slot >= 0) ? snap.recommended_slot : first_valid_slot(det);
    const bool has_phi = det.has_r_center() && (track_slot >= 0) && det.targets[track_slot].valid;
    const double phi_meas = has_phi ? det.targets[track_slot].angle : 0.0;

    double dt = has_last_meas_ ? (det.timestamp - last_timestamp_) : 0.0;
    if (has_phi) {
        update_direction_vote(phi_meas, dt);
        int dir_use = (dir_ != 0) ? dir_ : 1;

        // 常速状态下用 EKF 做相位平滑 (omega 强约束到 pi/3)
        if (det.robot_state.aim_mode == aimer::AimMode::ENERGY_SMALL ||
            det.robot_state.aim_mode == aimer::AimMode::ENERGY_LARGE) {
            update_constant_ekf(phi_meas, det.timestamp, dir_use * kOmegaConst);
        }

        // 大符激活状态: 最小二乘拟合 (lit_count==2)
        bool big_active_now = (det.robot_state.aim_mode == aimer::AimMode::ENERGY_LARGE) && (snap.lit_count >= 2);
        maybe_reset_big_fit(big_active_now, det.timestamp, phi_meas);
        if (big_active_now) {
            push_big_sample(det.timestamp, phi_meas);
            maybe_solve_big_fit();
        }
    }

    // 更新 last meas
    has_last_meas_ = has_phi;
    last_phi_meas_ = phi_meas;
    last_timestamp_ = det.timestamp;

    // 默认速度模型: 常速
    const int dir_use = (dir_ != 0) ? dir_ : 1;
    snap.model = SpeedModel::CONSTANT;
    snap.omega = dir_use * kOmegaConst;

    // 大符激活且拟合有效 -> BIG_SINE
    if (big_active_ && big_model_.valid) {
        snap.model = SpeedModel::BIG_SINE;
        snap.big = big_model_;
    }

    // ========== Group PnP: 估计 rune 平面与中心 ==========
    std::vector<cv::Point2f> img_pts;
    std::vector<cv::Point3f> obj_pts;
    img_pts.reserve(NUM_SLOTS + 1);
    obj_pts.reserve(NUM_SLOTS + 1);

    // R center as origin (optional)
    if (det.r_center.valid) {
        img_pts.push_back(det.r_center.center);
        obj_pts.emplace_back(0.f, 0.f, 0.f);
    }

    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!det.targets[i].valid) continue;
        img_pts.push_back(det.targets[i].center);
        double theta = i * 2.0 * M_PI / NUM_SLOTS;
        obj_pts.emplace_back(
            static_cast<float>(autobuff::RUNE_RADIUS * std::cos(theta)),
            static_cast<float>(autobuff::RUNE_RADIUS * std::sin(theta)),
            0.f
        );

        // Fill 2D-only slot info early
        snap.slots[i].valid = true;
        snap.slots[i].is_lit = (snap.lit_mask & (1u << i)) != 0;
        snap.slots[i].confidence = det.targets[i].confidence;
        snap.slots[i].center_px = det.targets[i].center;
        snap.slots[i].angle = det.targets[i].angle;
    }

    if (img_pts.size() < 4) {
        snap.valid = false;
        return snap;
    }

    const cv::Mat& K = aimer::tf::get_camera_matrix();
    const cv::Mat& D = aimer::tf::get_distort_coeffs();

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!ok) {
        snap.valid = false;
        return snap;
    }

    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);
    Eigen::Matrix3d R;
    R << Rcv.at<double>(0, 0), Rcv.at<double>(0, 1), Rcv.at<double>(0, 2),
         Rcv.at<double>(1, 0), Rcv.at<double>(1, 1), Rcv.at<double>(1, 2),
         Rcv.at<double>(2, 0), Rcv.at<double>(2, 1), Rcv.at<double>(2, 2);

    Eigen::Vector3d t(
        tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    snap.center_cam = t;
    snap.center_world = aimer::tf::cam_to_world(t, det.robot_state.q_imu);

    // Normal in camera frame: object +Z (wheel normal)
    Eigen::Vector3d n_cam = R.col(2);
    if (n_cam.z() < 0) n_cam = -n_cam;  // face camera (+z)
    snap.normal_cam = n_cam.normalized();

    // Fill 3D slot states
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!snap.slots[i].valid) continue;
        double theta = i * 2.0 * M_PI / NUM_SLOTS;
        Eigen::Vector3d p_obj(
            autobuff::RUNE_RADIUS * std::cos(theta),
            autobuff::RUNE_RADIUS * std::sin(theta),
            0.0
        );
        Eigen::Vector3d p_cam = R * p_obj + t;
        snap.slots[i].pos_cam = p_cam;
        snap.slots[i].pos_world = aimer::tf::cam_to_world(p_cam, det.robot_state.q_imu);
        snap.slots[i].vec_cam = p_cam - t;
    }

    snap.valid = true;
    return snap;
}

}  // namespace autobuff::predictor

