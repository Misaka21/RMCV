// LargeLsmModel: 大符激活 Ceres 最小二乘拟合模型 (2026)
// 优化: 二维网格搜索 + 线性 LS 粗估计 → Ceres 精修

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

    // 粗估计结果 (用于 Ceres warm start)
    struct CoarseResult {
        double a = 0.90;
        double w = 1.94;
        double tau = 0.0;
        double phi0 = 0.0;
        double residual = 1e9;
        bool valid = false;
    };

    // 核心拟合流程
    void solve_fit();

    // 阶段 1: 二维网格搜索 + 线性 LS (无局部最优)
    CoarseResult coarse_estimate(int dir_use) const;

    // 阶段 2: Ceres 精修 (用粗估计或上次结果初始化)
    bool ceres_refine(int dir_use, double a0, double w0, double tau0, double phi00,
                      int max_iter);

    static double reduced_angle(double x);
};

}  // namespace autobuff::predictor::models

#endif  // AIMER_AUTOBUFF_PREDICTOR_MODELS_LARGE_LSM_MODEL_HPP
