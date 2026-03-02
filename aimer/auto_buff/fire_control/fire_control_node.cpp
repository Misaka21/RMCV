/**
 * @file fire_control_node.cpp
 * @brief auto_buff fire control node (writes "fire_command")
 */

#include "fire_control_node.hpp"

#include <chrono>
#include <thread>

#include "aimer/auto_buff/predictor/types.hpp"
#include "aimer/common/latency/latency_estimator.hpp"
#include "aimer/common/trajectory/solver_factory.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "fire_controller.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"
#include "umt/BasicObjManager.hpp"

namespace autobuff::fire_control {

namespace {

double get_current_time() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

int choose_latency_slot(const autobuff::predictor::BuffSnapshot& snap) {
    if (!snap.valid) return -1;
    // Prefer recommended + lit
    if (snap.recommended_slot >= 0 && snap.is_lit(snap.recommended_slot)) {
        return snap.recommended_slot;
    }
    // Any lit
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (snap.is_lit(i)) return i;
    }
    // Fallback: any valid slot
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (snap.has_slot(i)) return i;
    }
    return -1;
}

::fire_control::LatencyInfo build_latency(
    const aimer::LatencyEstimator& estimator,
    const autobuff::predictor::BuffSnapshot& snap)
{
    double img_to_predict = (snap.predict_timestamp > 0.0)
        ? (snap.predict_timestamp - snap.timestamp)
        : 0.015;

    double distance = 5.0;
    int slot = choose_latency_slot(snap);
    if (snap.valid && slot >= 0 && snap.has_slot(slot)) {
        distance = snap.slots[slot].pos_cam.norm();
    }

    double bullet_speed = snap.self_state.bullet_speed;
    return estimator.build(img_to_predict, distance, bullet_speed, "AutoBuff.FireControl");
}

void finalize_latency(
    ::fire_control::LatencyInfo& latency,
    const autobuff::predictor::BuffSnapshot& snap,
    int slot)
{
    if (!snap.valid) return;
    if (slot < 0 || !snap.has_slot(slot)) return;

    constexpr int NUM_ITER = 2;
    for (int i = 0; i < NUM_ITER; ++i) {
        double dt = latency.prediction_latency();
        Eigen::Vector3d p = aimer::tf::cam_to_world(
            snap.predict_slot_cam(slot, dt), snap.self_state.q_imu);
        auto aim = ::fire_control::trajectory::solve(p, snap.self_state.bullet_speed);
        if (aim.valid) latency.set_fly_time(aim.fly_time);
    }
}

}  // namespace

void fire_control_run(const std::string& /*config_path*/) {
    debug::print(debug::PrintMode::INFO, "AutoBuffFireControl", "Starting...");

    auto snapshot_obj = umt::BasicObjManager<autobuff::predictor::BuffSnapshot>::find_or_create("buff_snapshot");
    auto fire_cmd = umt::BasicObjManager<::fire_control::FireCommand>::find_or_create("fire_command");
    auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    FireController controller;
    aimer::LatencyEstimator latency_estimator;

    aimer::AimMode last_mode = aimer::AimMode::DISABLED;
    int last_frame_id = -1;

    debug::print(debug::PrintMode::INFO, "AutoBuffFireControl", "Running at 500Hz");

    const auto period = std::chrono::microseconds(2000);
    auto next_time = std::chrono::steady_clock::now();

    while (app_running->get()) {
        watchdog::heartbeat("autobuff_fire_control");

        const auto& snap = snapshot_obj->get();
        double now = get_current_time();

        aimer::AimMode mode = snap.self_state.aim_mode;
        aimer::AimMode prev_mode = last_mode;

        // Update predict_to_send filter when a new frame arrives
        if (snap.frame_id != last_frame_id && snap.predict_timestamp > 0.0) {
            last_frame_id = snap.frame_id;
            double predict_to_send = now - snap.predict_timestamp;
            latency_estimator.update_predict_to_send(predict_to_send, now);
        }

        if (mode != last_mode) {
            debug::print(debug::PrintMode::INFO, "AutoBuffFireControl",
                         "Mode switch: {} -> {}",
                         aimer::aim_mode_name(last_mode),
                         aimer::aim_mode_name(mode));
            if (mode == aimer::AimMode::ENERGY_SMALL || mode == aimer::AimMode::ENERGY_LARGE) {
                controller.reset();
            }
            last_mode = mode;
        }

        ::fire_control::LatencyInfo latency = build_latency(latency_estimator, snap);
        int slot_for_latency = choose_latency_slot(snap);
        finalize_latency(latency, snap, slot_for_latency);

        ::fire_control::FireCommand cmd{};
        bool should_write = false;

        if (mode == aimer::AimMode::ENERGY_SMALL || mode == aimer::AimMode::ENERGY_LARGE) {
            cmd = controller.control(snap, now, latency);
            should_write = true;
        } else if (prev_mode == aimer::AimMode::ENERGY_SMALL || prev_mode == aimer::AimMode::ENERGY_LARGE) {
            // Leaving energy mode: send one disable command, then stop writing.
            cmd.control_enabled = false;
            should_write = true;
        }

        if (should_write) {
            fire_cmd->get() = cmd;

            // dashboard 输出 (供 webview 调试工具读取)
            int selected_rank = -1;
            if (cmd.target_id >= 0) {
                for (int r = 0; r < snap.ranked_count; ++r) {
                    if (snap.ccw_lit_rank[r] == cmd.target_id) {
                        selected_rank = r;
                        break;
                    }
                }
            }
            dashboard::set("buff_fire.selected_slot", cmd.target_id);
            dashboard::set("buff_fire.selected_rank", selected_rank);
            dashboard::set("buff_fire.tracking_error", static_cast<double>(cmd.tracking_error));
            dashboard::set("buff_fire.fire_now", cmd.fire_now);
            dashboard::set("buff_fire.coop_role",
                           runtime_param::get_param<std::string>("AutoBuff.FireControl.coop_role"));
        }

        next_time += period;
        std::this_thread::sleep_until(next_time);
    }

    debug::print(debug::PrintMode::INFO, "AutoBuffFireControl", "Stopped");
}

void start_fire_control_node(const std::string& config_path) {
    std::thread([config_path]() { fire_control_run(config_path); }).detach();
}

}  // namespace autobuff::fire_control

