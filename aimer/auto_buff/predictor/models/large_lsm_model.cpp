// LargeLsmModel 实现 (2026)
// 优化: 二维网格搜索 + 线性 LS 粗估计 → Ceres 精修

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

    if (start_time_ == 0.0) {
        start_time_ = timestamp;
        phi_unwrapped_ = phi_meas;
        last_phi_ = phi_meas;
        has_last_phi_ = true;
    }

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

    double window_sec =
        runtime_param::get_param<double>("AutoBuff.Predictor.LargeLSM.window_sec");
    if (window_sec <= 0.0) window_sec = 4.0;

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

// ============================================================================
// 阶段 1: 二维网格搜索 + 线性 LS
// ============================================================================
//
// 模型展开:
//   phi(t) = dir * (-(a/w)*cos(w*(t+tau)) + (2.090-a)*t) + phi0
//          = a * dir * (-cos(w*(t+tau))/w - t) + 2.090*dir*t + phi0
//          = a * k(t; w, tau) + c(t) + phi0
//
// 固定 (w, tau) 后，关于 [a, phi0] 是线性的。

LargeLsmModel::CoarseResult LargeLsmModel::coarse_estimate(int dir_use) const {
    CoarseResult best;
    if (samples_.size() < 10) return best;  // 数据太少

    // 网格配置 (w 范围窄，tau 范围宽)
    constexpr double W_MIN = 1.884, W_MAX = 2.000, W_STEP = 0.003;
    constexpr double TAU_MIN = -0.5, TAU_MAX = 0.5, TAU_STEP = 0.02;

    const int N = static_cast<int>(samples_.size());

    // 预提取样本到数组，避免反复访问 deque
    std::vector<double> t_arr(N), phi_arr(N);
    for (int i = 0; i < N; ++i) {
        t_arr[i] = samples_[i].first;
        phi_arr[i] = samples_[i].second;
    }

    for (double w = W_MIN; w <= W_MAX + 1e-6; w += W_STEP) {
        double iw = 1.0 / w;

        for (double tau = TAU_MIN; tau <= TAU_MAX + 1e-6; tau += TAU_STEP) {
            // 线性拟合: phi - c(t) = a * k(t) + phi0
            // 其中 c(t) = 2.090 * dir * t
            //       k(t) = dir * (-cos(w*(t+tau))/w - t)
            double sum_kk = 0.0, sum_k = 0.0, sum_ky = 0.0, sum_y = 0.0;

            for (int i = 0; i < N; ++i) {
                double t = t_arr[i];
                double c = 2.090 * dir_use * t;
                double k = dir_use * (-std::cos(w * (t + tau)) * iw - t);
                double y = phi_arr[i] - c;

                sum_kk += k * k;
                sum_k  += k;
                sum_ky += k * y;
                sum_y  += y;
            }

            double det = sum_kk * N - sum_k * sum_k;
            if (std::abs(det) < 1e-6) continue;

            double a = (sum_ky * N - sum_k * sum_y) / det;
            double phi0 = (sum_kk * sum_y - sum_k * sum_ky) / det;

            // 快速残差评估
            double res = 0.0;
            for (int i = 0; i < N; ++i) {
                double t = t_arr[i];
                double pred = dir_use * (-(a / w) * std::cos(w * (t + tau))
                                         + (2.090 - a) * t) + phi0;
                double e = pred - phi_arr[i];
                res += e * e;
            }

            if (res < best.residual) {
                best.residual = res;
                best.a = a;
                best.w = w;
                best.tau = tau;
                best.phi0 = phi0;
                best.valid = true;
            }
        }
    }

    return best;
}

// ============================================================================
// 阶段 2: Ceres 精修
// ============================================================================

bool LargeLsmModel::ceres_refine(int dir_use,
                                  double a0, double w0, double tau0, double phi00,
                                  int max_iter) {
    double p[4] = {a0, w0, tau0, phi00};

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
    options.max_num_iterations = max_iter;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        fit_valid_ = false;
        return false;
    }

    // RMS 残差检验
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
        return false;
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

    return true;
}

// ============================================================================
// 主拟合流程
// ============================================================================

void LargeLsmModel::solve_fit() {
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

    if (!fit_valid_) {
        // ========== 未收敛: 粗估计 → Ceres 精修 ==========
        auto coarse = coarse_estimate(dir_use);
        if (!coarse.valid) {
            fit_valid_ = false;
            return;
        }
        // 粗估计成功，用其结果初始化 Ceres，允许多次迭代
        ceres_refine(dir_use, coarse.a, coarse.w, coarse.tau, coarse.phi0, 25);
    } else {
        // ========== 已收敛: 局部精修 ==========
        // warm start，迭代次数少
        ceres_refine(dir_use, param_.a, param_.w, param_.tau, param_.phi0, 5);
    }
}

// ============================================================================
// 输出接口
// ============================================================================

MotionEstimate LargeLsmModel::estimate() const {
    MotionEstimate est;
    if (fit_valid_ && param_.valid) {
        est.model = SpeedModel::LARGE_SINE_LSM;
        est.large = param_;
        est.confidence = 1.0;
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
