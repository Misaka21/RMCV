// RuneObserver: PnP 几何解算 (2D 检测 → 3D 观测)

#ifndef AIMER_AUTOBUFF_PREDICTOR_OBSERVER_RUNE_OBSERVER_HPP
#define AIMER_AUTOBUFF_PREDICTOR_OBSERVER_RUNE_OBSERVER_HPP

#include "aimer/auto_buff/common/types.hpp"
#include "aimer/auto_buff/predictor/observer/rune_observation.hpp"

namespace autobuff::predictor {

class RuneObserver {
public:
    // 从 2D 检测结果 → 3D 观测
    RuneObservation observe(const BuffDetectionResult& det) const;
};

}  // namespace autobuff::predictor

#endif  // AIMER_AUTOBUFF_PREDICTOR_OBSERVER_RUNE_OBSERVER_HPP
