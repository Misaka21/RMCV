// ConstModel: 恒速模型 (小符/大符非激活兜底) (2026)

#ifndef AIMER_AUTOBUFF_PREDICTOR_MODELS_CONST_MODEL_HPP
#define AIMER_AUTOBUFF_PREDICTOR_MODELS_CONST_MODEL_HPP

#include "model_interface.hpp"

namespace autobuff::predictor::models {

// 固定 omega = π/3 rad/s 的恒速模型
class ConstModel final : public MotionModelInterface {
public:
    ConstModel() = default;

    void reset() override;
    void feed(double phi_meas, double timestamp, int dir_sign) override;
    MotionEstimate estimate() const override;
    bool ready() const override;

private:
    int dir_sign_ = 0;
};

}  // namespace autobuff::predictor::models

#endif  // AIMER_AUTOBUFF_PREDICTOR_MODELS_CONST_MODEL_HPP
