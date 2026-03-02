// Energy rune predictor (2026) - 重构版

#ifndef AIMER_AUTOBUFF_PREDICTOR_BUFF_PREDICTOR_HPP
#define AIMER_AUTOBUFF_PREDICTOR_BUFF_PREDICTOR_HPP

#include "aimer/auto_buff/common/types.hpp"
#include "aimer/auto_buff/predictor/direction_estimator.hpp"
#include "aimer/auto_buff/predictor/mode_manager.hpp"
#include "aimer/auto_buff/predictor/models/const_model.hpp"
#include "aimer/auto_buff/predictor/models/large_lsm_model.hpp"
#include "aimer/auto_buff/predictor/models/small_ekf_model.hpp"
#include "aimer/auto_buff/predictor/slot_debouncer.hpp"
#include "aimer/auto_buff/predictor/types.hpp"

namespace autobuff::predictor {

class BuffPredictor {
public:
    BuffPredictor() = default;

    void reset();
    BuffSnapshot predict(const BuffDetectionResult& det);

private:
    SlotDebouncer debouncer_;
    DirectionEstimator dir_estimator_;
    ModeManager mode_mgr_;

    models::ConstModel const_model_;
    models::SmallEkfModel small_model_;
    models::LargeLsmModel large_model_;

    int last_track_slot_ = -1;
    double last_track_phi_ = 0.0;
    double last_timestamp_ = 0.0;
    bool has_last_track_ = false;

    int choose_track_slot(
        const SlotDebouncer::Output& debounced,
        const BuffDetectionResult& det) const;

    void build_ccw_rank(BuffSnapshot& snap) const;

    static double reduced_angle(double x);
};

}  // namespace autobuff::predictor

#endif  // AIMER_AUTOBUFF_PREDICTOR_BUFF_PREDICTOR_HPP
