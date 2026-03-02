// ConstModel 实现 (2026)

#include "const_model.hpp"

#include <cmath>

namespace autobuff::predictor::models {

static constexpr double kOmegaConst = M_PI / 3.0;  // π/3 rad/s

void ConstModel::reset() {
    dir_sign_ = 0;
}

void ConstModel::feed(double /*phi_meas*/, double /*timestamp*/, int dir_sign) {
    if (dir_sign != 0) {
        dir_sign_ = dir_sign;
    }
}

MotionEstimate ConstModel::estimate() const {
    MotionEstimate est;
    est.model = SpeedModel::CONST_OMEGA;
    int d = (dir_sign_ != 0) ? dir_sign_ : 1;
    est.omega_signed = d * kOmegaConst;
    est.confidence = (dir_sign_ != 0) ? 0.8 : 0.3;
    return est;
}

bool ConstModel::ready() const {
    return true;  // 恒速模型始终就绪 (只要有方向符号)
}

}  // namespace autobuff::predictor::models
