// Energy rune predictor (2026)

#ifndef AIMER_AUTOBUFF_PREDICTOR_BUFF_PREDICTOR_HPP
#define AIMER_AUTOBUFF_PREDICTOR_BUFF_PREDICTOR_HPP

#include <deque>
#include <utility>

#include "aimer/auto_buff/common/types.hpp"
#include "aimer/auto_buff/predictor/types.hpp"
#include "aimer/common/filter/adaptive_ekf.hpp"

namespace autobuff::predictor {

class BuffPredictor {
public:
    BuffPredictor() = default;

    BuffSnapshot predict(const BuffDetectionResult& det);

private:
    // Direction estimation (sign of dphi)
    int dir_ = 0;           // -1/0/+1
    int dir_votes_ = 0;     // accum votes
    double last_phi_meas_ = 0.0;
    double last_timestamp_ = 0.0;
    bool has_last_meas_ = false;

    // Constant-speed EKF: x=[phi, omega]
    bool ekf_inited_ = false;
    aimer::filter::AdaptiveEkf<2, 1> ekf_;

    // Big active fitting buffer (t, phi_unwrapped)
    bool big_active_ = false;
    double big_start_time_ = 0.0;
    double big_phi_unwrapped_ = 0.0;
    double big_last_phi_ = 0.0;
    bool big_has_last_phi_ = false;
    std::deque<std::pair<double, double>> big_samples_;
    BigSineModel big_model_;

    // Helpers
    static int sgn(double x);
    static double reduced_angle(double x);
    static double closest_angle(double target, double current);

    void update_direction_vote(double phi_meas, double dt);
    void update_constant_ekf(double phi_meas, double t, double omega_guess);

    void maybe_reset_big_fit(bool big_active_now, double timestamp, double phi_meas);
    void push_big_sample(double timestamp, double phi_meas);
    void maybe_solve_big_fit();
};

}  // namespace autobuff::predictor

#endif  // AIMER_AUTOBUFF_PREDICTOR_BUFF_PREDICTOR_HPP

