// 运动模型接口 (2026)

#ifndef AIMER_AUTOBUFF_PREDICTOR_MODELS_MODEL_INTERFACE_HPP
#define AIMER_AUTOBUFF_PREDICTOR_MODELS_MODEL_INTERFACE_HPP

#include "aimer/auto_buff/predictor/types.hpp"

namespace autobuff::predictor::models {

class MotionModelInterface {
public:
    virtual ~MotionModelInterface() = default;
    virtual void reset() = 0;

    // 喂入观测: phi_meas 弧度, timestamp 秒, dir_sign +1/-1/0
    virtual void feed(double phi_meas, double timestamp, int dir_sign) = 0;

    virtual MotionEstimate estimate() const = 0;

    // 模型是否已收敛到可用状态
    virtual bool ready() const = 0;
};

}  // namespace autobuff::predictor::models

#endif  // AIMER_AUTOBUFF_PREDICTOR_MODELS_MODEL_INTERFACE_HPP
