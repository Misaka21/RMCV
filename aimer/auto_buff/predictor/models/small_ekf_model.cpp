// SmallEkfModel 实现 (2026)

#include "small_ekf_model.hpp"

#include <cmath>

#include "plugin/param/runtime_parameter.hpp"

namespace autobuff::predictor::models {

static constexpr double kOmegaConst = M_PI / 3.0;  // π/3 rad/s

// EKF 预测仿函数: phi' = phi + omega*dt, omega' = omega
struct SmallEkfPredictFunc {
    double dt;
    template <typename T>
    void operator()(const T x_in[2], T x_out[2]) const {
        x_out[0] = x_in[0] + T(dt) * x_in[1];
        x_out[1] = x_in[1];
    }
};

// EKF 观测仿函数: observe phi
struct SmallEkfMeasureFunc {
    template <typename T>
    void operator()(const T x[2], T y[1]) const {
        y[0] = x[0];
    }
};

double SmallEkfModel::closest_angle(double target, double current) {
    double diff = target - current;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return current + diff;
}

void SmallEkfModel::reset() {
    inited_ = false;
    last_timestamp_ = 0.0;
    ekf_ = aimer::filter::AdaptiveEkf<2, 1>{};
}

void SmallEkfModel::feed(double phi_meas, double timestamp, int dir_sign) {
    using Ekf = aimer::filter::AdaptiveEkf<2, 1>;
    using MatrixXX = Ekf::MatrixXX;
    using MatrixYY = Ekf::MatrixYY;
    using VectorX = Ekf::VectorX;
    using VectorY = Ekf::VectorY;

    int d = (dir_sign != 0) ? dir_sign : 1;
    double omega_guess = d * kOmegaConst;

    if (!inited_) {
        VectorX x0;
        x0 << phi_meas, omega_guess;
        ekf_.init(x0);
        inited_ = true;
        last_timestamp_ = timestamp;
        return;
    }

    double dt = timestamp - last_timestamp_;
    if (dt < 1e-4 || dt > 0.2) {
        // 时间戳异常，重置
        VectorX x0;
        x0 << phi_meas, omega_guess;
        ekf_.init(x0);
        last_timestamp_ = timestamp;
        return;
    }

    // 在使用点直接读取运行时参数 (禁止缓存)
    double q_phi   = runtime_param::get_param<double>("AutoBuff.Predictor.SmallEKF.q_phi");
    double q_omega = runtime_param::get_param<double>("AutoBuff.Predictor.SmallEKF.q_omega");
    double r_phi   = runtime_param::get_param<double>("AutoBuff.Predictor.SmallEKF.r_phi");

    if (q_phi <= 0.0)   q_phi   = 2e-4;
    if (q_omega <= 0.0) q_omega = 5e-3;
    if (r_phi <= 0.0)   r_phi   = 4e-3;

    MatrixXX Q = MatrixXX::Zero();
    Q(0, 0) = q_phi;
    Q(1, 1) = q_omega;
    ekf_.predict_forward_scaled(SmallEkfPredictFunc{dt}, Q);

    // 角度展开到最近邻
    VectorX x_pred = ekf_.get_x();
    double phi_adj = closest_angle(phi_meas, x_pred[0]);

    // 检查 omega 是否飘离 → 重置
    double omega_ekf = x_pred[1];
    if (std::abs(omega_ekf - d * kOmegaConst) > M_PI / 6.0) {
        VectorX x0;
        x0 << phi_meas, omega_guess;
        ekf_.init(x0);
        last_timestamp_ = timestamp;
        return;
    }

    VectorY y;
    y << phi_adj;
    MatrixYY R = MatrixYY::Identity() * r_phi;
    ekf_.update_forward(SmallEkfMeasureFunc{}, y, R);

    last_timestamp_ = timestamp;
}

MotionEstimate SmallEkfModel::estimate() const {
    MotionEstimate est;
    est.model = SpeedModel::CONST_OMEGA;
    if (inited_) {
        est.omega_signed = ekf_.get_x()[1];
        est.confidence = 0.9;
    } else {
        est.omega_signed = kOmegaConst;
        est.confidence = 0.0;
    }
    return est;
}

bool SmallEkfModel::ready() const {
    return inited_;
}

}  // namespace autobuff::predictor::models
