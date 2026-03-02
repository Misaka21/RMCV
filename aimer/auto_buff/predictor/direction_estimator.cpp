// DirectionEstimator 实现 (2026)

#include "direction_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace autobuff::predictor {

static inline double reduced_angle(double x) {
    return std::atan2(std::sin(x), std::cos(x));
}

static inline int sgn(double x) {
    return (x > 0) - (x < 0);
}

void DirectionEstimator::feed(double phi_now, double phi_last, double dt) {
    if (dt < 1e-3) return;

    double dphi = reduced_angle(phi_now - phi_last);
    if (std::abs(dphi) < 1e-4) return;

    int v = sgn(dphi);
    votes_ += v;
    votes_ = std::clamp(votes_, -20, 20);

    if (std::abs(votes_) >= 8) {
        dir_ = sgn(static_cast<double>(votes_));
    }
}

autobuff::RotateDir DirectionEstimator::direction() const {
    if (dir_ > 0) return autobuff::RotateDir::CCW;
    if (dir_ < 0) return autobuff::RotateDir::CW;
    return autobuff::RotateDir::UNKNOWN;
}

int DirectionEstimator::dir_sign() const {
    return dir_;
}

void DirectionEstimator::reset() {
    votes_ = 0;
    dir_ = 0;
}

}  // namespace autobuff::predictor
