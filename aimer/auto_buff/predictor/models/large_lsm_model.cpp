// LargeLsmModel 实现 (2026)

#include "large_lsm_model.hpp"

#include <algorithm>
#include <cmath>

#include <ceres/ceres.h>

#include "plugin/param/runtime_parameter.hpp"

namespace autobuff::predictor::models {

// 大符相位残差 (Ceres AutoDiff)
struct LargePhiResidual {
    LargePhiResidual(double t, double y, int dir) : t_(t), y_(y), dir_(dir) {}

    template <typename T>
    bool operator()(const T* const p, T* residual) const {
        // p = [a, w, tau, phi0]
        const T a    = p[0];
        const T w    = p[1];
        const T tau  = p[2];
        const T phi0 = p[3];

        const T b = T(2.090) - a;
        const T t = T(t_);

        const T pred = T(dir_) * (-(a / w) * ceres::cos(w * (t + tau)) + b * t) + phi0;
        residual[0] = pred - T(y_);
        return true;
    }

    double t_;
    double y_;
    int dir_;
};

double LargeLsmModel::reduced_angle(double x) {
    return std::atan2(std::sin(x), std::cos(x));
}

void LargeLsmModel::reset() {
    samples_.clear();
    start_time_ = 0.0;
    phi_unwrapped_ = 0.0;
    last_phi_ = 0.0;
    has_last_phi_ = false;
    dir_sign_ = 0;
    param_ = LargeSineParam{};
    fit_valid_ = false;
}

void LargeLsmModel::feed(double phi_meas, double timestamp, int dir_sign) {
    if (dir_sign != 0) dir_sign_ = dir_sign;

    // 初始化起始时间
    if (start_time_ == 0.0) {
        start_time_ = timestamp;
        phi_unwrapped_ = phi_meas;
        last_phi_ = phi_meas;
        has_last_phi_ = true;
    }

    // 相位展开
    if (!has_last_phi_) {
        phi_unwrapped_ = phi_meas;
        last_phi_ = phi_meas;
        has_last_phi_ = true;
    } else {
        phi_unwrapped_ += reduced_angle(phi_meas - last_phi_);
        last_phi_ = phi_meas;
    }

    double t_rel = timestamp - start_time_;
    samples_.emplace_back(t_rel, phi_unwrapped_);

    // 在使用点直接读取运行时参数 (禁止缓存)
    double window_sec =
        runtime_param::get_param<double>("AutoBuff.Predictor.LargeLSM.window_sec");
    if (window_sec <= 0.0) window_sec = 4.0;

    // 滑窗裁剪
    constexpr size_t MAX_SAMPLES = 250;
    double t_min = t_rel - window_sec;
    while (!samples_.empty() && samples_.front().first < t_min) {
        samples_.pop_front();
    }
    while (samples_.size() > MAX_SAMPLES) {
        samples_.pop_front();
    }

    solve_fit();
}

void LargeLsmModel::solve_fit() {
    // 在使用点直接读取运行时参数 (禁止缓存)
    int min_samples = static_cast<int>(
        runtime_param::get_param<int64_t>("AutoBuff.Predictor.LargeLSM.min_samples"));
    if (min_samples <= 0) min_samples = 35;

    double min_span_sec =
        runtime_param::get_param<double>("AutoBuff.Predictor.LargeLSM.min_span_sec");
    if (min_span_sec <= 0.0) min_span_sec = 0.6;

    if (static_cast<int>(samples_.size()) < min_samples) return;

    double t_span = samples_.back().first - samples_.front().first;
    if (t_span < min_span_sec) return;

    int dir_use = (dir_sign_ != 0) ? dir_sign_ : 1;

    // 暖启动: 如果上次有效，继续使用上次结果为初始值
    double p[4] = {
        fit_valid_ ? param_.a    : 0.90,
        fit_valid_ ? param_.w    : 1.94,
        fit_valid_ ? param_.tau  : 0.0,
        fit_valid_ ? param_.phi0 : samples_.front().second
    };

    ceres::Problem problem;
    for (const auto& [t, y] : samples_) {
        auto* cost = new ceres::AutoDiffCostFunction<LargePhiResidual, 1, 4>(
            new LargePhiResidual(t, y, dir_use));
        double huber_delta =
            runtime_param::get_param<double>("AutoBuff.Predictor.LargeLSM.huber_delta");
        if (huber_delta <= 0.0) huber_delta = 0.1;
        problem.AddResidualBlock(cost, new ceres::HuberLoss(huber_delta), p);
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
        fit_valid_ = false;
        return;
    }

    // 计算 RMS 残差
    double rms = 0.0;
    for (const auto& [t, y] : samples_) {
        LargeSineParam tmp;
        tmp.dir = dir_use;
        tmp.start_time = start_time_;
        tmp.a = p[0]; tmp.w = p[1]; tmp.tau = p[2]; tmp.phi0 = p[3];
        double e = tmp.phi(t) - y;
        rms += e * e;
    }
    rms = std::sqrt(rms / static_cast<double>(samples_.size()));

    double residual_accept =
        runtime_param::get_param<double>("AutoBuff.Predictor.LargeLSM.residual_accept");
    if (residual_accept <= 0.0) residual_accept = 0.18;

    if (rms > residual_accept) {
        fit_valid_ = false;
        return;
    }

    fit_valid_ = true;
    param_.valid        = true;
    param_.dir          = dir_use;
    param_.start_time   = start_time_;
    param_.a            = p[0];
    param_.w            = p[1];
    param_.tau          = p[2];
    param_.phi0         = p[3];
    param_.residual_rms = rms;
    param_.sample_count = static_cast<int>(samples_.size());
}

MotionEstimate LargeLsmModel::estimate() const {
    MotionEstimate est;
    if (fit_valid_ && param_.valid) {
        est.model = SpeedModel::LARGE_SINE_LSM;
        est.large = param_;
        est.confidence = 1.0;
        // omega_signed 用于显示: 大符瞬时平均速度 b = 2.090 - a
        est.omega_signed = param_.dir * param_.b();
    } else {
        est.model = SpeedModel::UNKNOWN;
        est.confidence = 0.0;
    }
    return est;
}

bool LargeLsmModel::ready() const {
    return fit_valid_ && param_.valid;
}

}  // namespace autobuff::predictor::models
