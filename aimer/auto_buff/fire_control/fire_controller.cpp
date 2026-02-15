// auto_buff fire controller implementation (2026)

#include "fire_controller.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "aimer/common/trajectory/solver_factory.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autobuff::fire_control {

namespace {

constexpr int MAX_LOST_COUNT = 50;

inline double get_param_or(const std::string& key, double fallback) {
    double v = runtime_param::get_param<double>(key);
    return (v > 0) ? v : fallback;
}

}  // namespace

void FireController::reset() {
    last_time_ = 0.0;
    lost_count_ = 0;
}

double FireController::normalize_angle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

double FireController::tracking_error(const ::fire_control::AimResult& aim, const ::fire_control::GimbalState& g) {
    double dy = normalize_angle(aim.yaw - g.yaw);
    double dp = (aim.pitch - g.pitch);
    return std::hypot(dy, dp);
}

::fire_control::FireCommand FireController::control(
    const autobuff::predictor::BuffSnapshot& snapshot,
    double current_time,
    const ::fire_control::LatencyInfo& latency)
{
    // 1) update gimbal state
    double dt = (last_time_ > 0.0) ? (current_time - last_time_) : ::fire_control::DEFAULT_CONTROL_DT;
    gimbal_state_.update(snapshot.self_state.q_imu, dt);
    last_time_ = current_time;

    // 2) basic validity
    if (!snapshot.valid) {
        if (++lost_count_ > MAX_LOST_COUNT) reset();
        return no_target_command();
    }
    lost_count_ = 0;

    // 3) collect lit candidates
    std::vector<int> lit_ids;
    lit_ids.reserve(NUM_SLOTS);
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (snapshot.is_lit(i)) lit_ids.push_back(i);
    }
    if (lit_ids.empty()) return no_target_command();

    // 4) choose best slot (large: pick lower tracking error)
    const double predict_dt = latency.prediction_latency();
    const double bullet_speed = snapshot.self_state.bullet_speed;

    int best_slot = -1;
    ::fire_control::AimResult best_aim{};
    double best_err = 1e9;

    if (snapshot.self_state.aim_mode == aimer::AimMode::ENERGY_SMALL) {
        best_slot = lit_ids.front();
        Eigen::Vector3d p = snapshot.predict_slot_world(best_slot, predict_dt);
        best_aim = ::fire_control::trajectory::solve(p, bullet_speed);
        if (!best_aim.valid) return no_target_command();
        best_err = tracking_error(best_aim, gimbal_state_);
    } else {
        // ENERGY_LARGE: evaluate each lit
        for (int slot : lit_ids) {
            Eigen::Vector3d p = snapshot.predict_slot_world(slot, predict_dt);
            auto aim = ::fire_control::trajectory::solve(p, bullet_speed);
            if (!aim.valid) continue;
            double err = tracking_error(aim, gimbal_state_);
            if (err < best_err) {
                best_err = err;
                best_slot = slot;
                best_aim = aim;
            }
        }
        if (best_slot < 0 || !best_aim.valid) return no_target_command();
    }

    // 5) fire decision
    double fire_threshold = get_param_or("AutoBuff.FireControl.fire_threshold", 0.02);
    double min_conf = get_param_or("AutoBuff.FireControl.min_confidence", 0.30);

    float conf = snapshot.slots[best_slot].confidence;
    bool can_fire = snapshot.self_state.allow_fire
        && (static_cast<double>(conf) >= min_conf)
        && (best_err <= fire_threshold);

    // 6) command
    ::fire_control::FireCommand cmd;
    cmd.control_enabled = true;
    cmd.yaw = static_cast<float>(best_aim.yaw);
    cmd.pitch = static_cast<float>(best_aim.pitch);
    cmd.allow_fire = true;
    cmd.fire_now = can_fire;

    cmd.target_id = best_slot;
    cmd.tracking_error = static_cast<float>(best_err);
    cmd.confidence = conf;
    return cmd;
}

::fire_control::FireCommand FireController::no_target_command() const {
    ::fire_control::FireCommand cmd;
    cmd.control_enabled = false;
    cmd.allow_fire = false;
    cmd.fire_now = false;
    cmd.target_id = -1;
    return cmd;
}

}  // namespace autobuff::fire_control

