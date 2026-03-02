// SmallEkfModel: 小符 EKF 运动模型 (恒速 + EKF 平滑) (2026)

#ifndef AIMER_AUTOBUFF_PREDICTOR_MODELS_SMALL_EKF_MODEL_HPP
#define AIMER_AUTOBUFF_PREDICTOR_MODELS_SMALL_EKF_MODEL_HPP

#include "model_interface.hpp"
#include "aimer/common/filter/adaptive_ekf.hpp"

namespace autobuff::predictor::models {

// 状态: [phi, omega], 观测: [phi]
class SmallEkfModel final : public MotionModelInterface {
public:
    SmallEkfModel() = default;

    void reset() override;
    void feed(double phi_meas, double timestamp, int dir_sign) override;
    MotionEstimate estimate() const override;
    bool ready() const override;

private:
    aimer::filter::AdaptiveEkf<2, 1> ekf_;
    bool inited_ = false;
    double last_timestamp_ = 0.0;

    static double closest_angle(double target, double current);
};

}  // namespace autobuff::predictor::models

#endif  // AIMER_AUTOBUFF_PREDICTOR_MODELS_SMALL_EKF_MODEL_HPP
