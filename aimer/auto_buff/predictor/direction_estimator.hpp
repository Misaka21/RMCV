// DirectionEstimator: 集中化的旋转方向投票 (2026)

#ifndef AIMER_AUTOBUFF_PREDICTOR_DIRECTION_ESTIMATOR_HPP
#define AIMER_AUTOBUFF_PREDICTOR_DIRECTION_ESTIMATOR_HPP

#include "aimer/auto_buff/common/types.hpp"

namespace autobuff::predictor {

class DirectionEstimator {
public:
    DirectionEstimator() = default;

    // 喂入新的相位观测，更新投票计数
    // phi_now/phi_last: 弧度 (相对 R 中心角度)
    // dt: 帧间时间 (s)
    void feed(double phi_now, double phi_last, double dt);

    autobuff::RotateDir direction() const;

    // 返回 +1 (CCW) / -1 (CW) / 0 (未知)
    int dir_sign() const;

    void reset();

private:
    int votes_ = 0;    // 累计投票 [-20, +20]
    int dir_ = 0;      // 已锁定方向符号 -1/0/+1
};

}  // namespace autobuff::predictor

#endif  // AIMER_AUTOBUFF_PREDICTOR_DIRECTION_ESTIMATOR_HPP
