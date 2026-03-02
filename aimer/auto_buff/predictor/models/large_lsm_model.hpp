// LargeLsmModel: 大符激活 Ceres 最小二乘拟合模型 (2026)

#ifndef AIMER_AUTOBUFF_PREDICTOR_MODELS_LARGE_LSM_MODEL_HPP
#define AIMER_AUTOBUFF_PREDICTOR_MODELS_LARGE_LSM_MODEL_HPP

#include <deque>
#include <utility>

#include "model_interface.hpp"

namespace autobuff::predictor::models {

class LargeLsmModel final : public MotionModelInterface {
public:
    LargeLsmModel() = default;

    void reset() override;
    void feed(double phi_meas, double timestamp, int dir_sign) override;
    MotionEstimate estimate() const override;
    bool ready() const override;

private:
    // 滑窗样本 (t_rel, phi_unwrapped)
    std::deque<std::pair<double, double>> samples_;

    double start_time_ = 0.0;  // 0 表示未初始化
    double phi_unwrapped_ = 0.0;
    double last_phi_ = 0.0;
    bool has_last_phi_ = false;

    int dir_sign_ = 0;

    LargeSineParam param_{};
    bool fit_valid_ = false;

    void solve_fit();
    static double reduced_angle(double x);
};

}  // namespace autobuff::predictor::models

#endif  // AIMER_AUTOBUFF_PREDICTOR_MODELS_LARGE_LSM_MODEL_HPP
