// auto_buff fire controller (2026)

#ifndef AIMER_AUTOBUFF_FIRE_CONTROL_FIRE_CONTROLLER_HPP
#define AIMER_AUTOBUFF_FIRE_CONTROL_FIRE_CONTROLLER_HPP

#include "aimer/auto_buff/fire_control/coop_policy.hpp"
#include "aimer/auto_buff/fire_control/target_ranker.hpp"
#include "aimer/auto_buff/predictor/types.hpp"
#include "aimer/common/fire_control_types.hpp"

namespace autobuff::fire_control {

class FireController {
public:
    void reset();

    ::fire_control::FireCommand control(
        const autobuff::predictor::BuffSnapshot& snapshot,
        double current_time,
        const ::fire_control::LatencyInfo& latency);

private:
    TargetRanker ranker_;
    CoopPolicy coop_;
    ::fire_control::GimbalState gimbal_state_;
    double last_time_ = 0.0;
    int lost_count_ = 0;

    static double normalize_angle(double a);
    ::fire_control::FireCommand no_target_command() const;
};

}  // namespace autobuff::fire_control

#endif  // AIMER_AUTOBUFF_FIRE_CONTROL_FIRE_CONTROLLER_HPP
