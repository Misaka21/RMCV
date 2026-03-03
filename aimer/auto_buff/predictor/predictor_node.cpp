/**
 * @file predictor_node.cpp
 * @brief auto_buff 预测器节点
 *
 * Subscribe : Message<BuffDetectionResult> "buff_detections"
 * Output    : BasicObjManager<BuffSnapshot> "buff_snapshot"
 */

#include "predictor_node.hpp"

#include <chrono>
#include <thread>

#include "buff_predictor.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"
#include "umt/umt.hpp"

namespace autobuff::predictor {

using SteadyClock = std::chrono::steady_clock;

void start_predictor_node() {
    debug::print(debug::PrintMode::INFO, "BuffPredictorNode", "Starting...");

    BuffPredictor predictor;

    umt::Subscriber<autobuff::BuffDetectionResult> sub("buff_detections");
    auto snapshot_obj = umt::BasicObjManager<BuffSnapshot>::find_or_create("buff_snapshot");
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    stats::FpsStats stats("BuffPredictorNode", "tracked");

    bool was_active = false;
    constexpr double INACTIVE_RESET_DELAY_S = 0.4;
    double inactive_since_s = -1.0;

    auto now_seconds = []() -> double {
        return std::chrono::duration<double>(
            SteadyClock::now().time_since_epoch()).count();
    };
    auto mark_inactive = [&](double t_now_s) {
        if (inactive_since_s < 0.0) {
            inactive_since_s = t_now_s;
        }
        if ((t_now_s - inactive_since_s) > INACTIVE_RESET_DELAY_S) {
            was_active = false;
        }
    };

    while (running->get()) {
        watchdog::heartbeat("buff_predictor");
        try {
            auto det = sub.pop_for(1000);
            double t_now_s = now_seconds();

            // 仅在能量机关模式下工作
            if (det.robot_state.aim_mode != aimer::AimMode::ENERGY_SMALL &&
                det.robot_state.aim_mode != aimer::AimMode::ENERGY_LARGE) {
                mark_inactive(t_now_s);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            inactive_since_s = -1.0;
            // 从非活跃状态重新进入: 重置 predictor 清除旧数据
            if (!was_active) {
                predictor.reset();
                was_active = true;
            }

            auto t0 = SteadyClock::now();
            auto snap = predictor.predict(det);
            auto t1 = SteadyClock::now();

            float latency_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0f;

            // predictor finish timestamp (供 LatencyEstimator 估计 predict_to_send)
            auto since_epoch = t1.time_since_epoch();
            snap.predict_timestamp = std::chrono::duration<double>(since_epoch).count();

            snapshot_obj->get() = snap;

            stats.update(latency_ms, snap.valid);
            dashboard::set("buff_predictor.latency_ms", latency_ms);
            dashboard::set("buff_predictor.fps", stats.last_fps);
            dashboard::set("buff_predictor.valid", snap.valid ? 1 : 0);
            dashboard::set("buff_predictor.lit_count", snap.lit_count);
            dashboard::set("buff_predictor.omega", snap.motion.omega_signed);
            dashboard::set("buff_predictor.model", static_cast<int>(snap.motion.model));
            dashboard::set("buff_predictor.mode", static_cast<int>(snap.mode));
            dashboard::set("buff_predictor.direction", static_cast<int>(snap.direction));

        } catch (const umt::MessageError_Timeout&) {
            mark_inactive(now_seconds());
        } catch (const umt::MessageError_Stopped&) {
            mark_inactive(now_seconds());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "BuffPredictorNode",
                         "Exception: {}", e.what());
        }
    }

    debug::print(debug::PrintMode::INFO, "BuffPredictorNode", "Stopped");
}

}  // namespace autobuff::predictor
