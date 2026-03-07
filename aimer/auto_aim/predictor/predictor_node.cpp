/**
 * @file predictor_node.cpp
 * @brief 预测器节点
 *
 * 订阅: Message<DetectionResult> "detections"
 * 输出: BasicObjManager<BattlefieldSnapshot> "battlefield"
 */

#include <chrono>
#include <string>
#include <thread>

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

#include "aimer/common/types.hpp"
#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "enemy_predictor.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"
#include "plugin/plotter/plotter.hpp"
#include "umt/umt.hpp"

namespace autoaim::predictor {

using SteadyClock = std::chrono::steady_clock;

namespace {

bool get_runtime_bool_or(const std::string& name, bool default_value) {
    auto ptr = runtime_param::find_param(name);
    if (ptr == nullptr) return default_value;
    if (auto* val = std::get_if<bool>(&*ptr)) return *val;
    return default_value;
}

std::string get_runtime_string_or(const std::string& name, std::string default_value) {
    auto ptr = runtime_param::find_param(name);
    if (ptr == nullptr) return default_value;
    if (auto* val = std::get_if<std::string>(&*ptr)) return *val;
    return default_value;
}

}  // namespace

/**
 * @brief 启动预测器节点
 */
void start_predictor_node() {
    debug::print(debug::PrintMode::INFO, "PredictorNode", "Starting predictor node...");

    // 创建预测器
    EnemyPredictor predictor;

    // 设置 UMT
    umt::Subscriber<DetectionResult> sub("detections");
    auto battlefield = umt::BasicObjManager<BattlefieldSnapshot>::find_or_create("battlefield");
    auto predictor_debug =
        umt::BasicObjManager<PredictorDebugFrame>::find_or_create("predictor_debug");
    umt::Publisher<cv::Mat> pub_debug("/predictor/debug");  // Web 调试图像
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    stats::FpsStats stats("PredictorNode", "tracked");

    debug::print(debug::PrintMode::INFO, "PredictorNode", "Predictor node started");

    while (running->get()) {
        watchdog::heartbeat("predictor");

        try {
            auto detection = sub.pop_for(1000);
            auto t_pop = SteadyClock::now();

            // 非自瞄模式时跳过预测
            if (detection.state.aim_mode != aimer::AimMode::AUTOAIM) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // 使用相机帧时间戳（从 SyncFrame 传递过来，微秒转秒）
            double timestamp = detection.state.timestamp_us / 1e6;

            // [阶段2] EKF 预测
            auto snapshot = predictor.predict(detection, timestamp);
            auto t_predict = SteadyClock::now();

            float latency = std::chrono::duration_cast<std::chrono::microseconds>(
                t_predict - t_pop).count() / 1000.0f;

            // 设置预测完成时间戳 (供火控计算 predict_to_send 延迟)
            auto predict_time_since_epoch = t_predict.time_since_epoch();
            snapshot.predict_timestamp = std::chrono::duration<double>(predict_time_since_epoch).count();

            // [阶段3] 可视化数据准备
            // 默认不在预测线程做重绘，避免显示负载污染预测时序。
            bool show_window = get_runtime_bool_or("Visualizer.show_window", false);
            std::string view = show_window
                ? get_runtime_string_or("Visualizer.view", "")
                : std::string{};
            bool need_predictor_view = show_window && view != "detector";
            bool need_vis = need_predictor_view || pub_debug.has_subscriber();

            PredictorDebugFrame debug_frame;
            debug_frame.frame_id = snapshot.frame_id;
            debug_frame.timestamp = snapshot.timestamp;
            debug_frame.q_imu = snapshot.self_state.q_imu;
            debug_frame.detect_latency_ms = detection.latency_ms;
            debug_frame.predict_latency_ms = latency;

            if (need_vis && !detection.img.empty()) {
                debug_frame.image = detection.img.clone();
                predictor.draw(debug_frame.image, detection.state.q_imu, timestamp);
            }

            // [阶段4] 写入共享对象 (线程安全)
            battlefield->store(snapshot);
            predictor_debug->store(debug_frame);

            // Web 调试图像
            if (pub_debug.has_subscriber() && !debug_frame.image.empty()) {
                pub_debug.push(debug_frame.image);
            }

            // [阶段5] 输出云台状态到 PlotJuggler
            {
                const auto& R_g2i = aimer::tf::Transform<
                    aimer::tf::Frame::Gimbal, aimer::tf::Frame::Imu>::R_;
                Eigen::Quaterniond q_gimbal(snapshot.self_state.q_imu.toRotationMatrix() * R_g2i);
                auto [yaw, pitch] = aimer::math::quat_to_yaw_pitch(q_gimbal);
                plotter::begin();
                plotter::add("/gimbal/yaw", yaw * 57.3);
                plotter::add("/gimbal/pitch", pitch * 57.3);
                plotter::end();
            }

            // [阶段6] 统计 + Dashboard
            int tracked = 0;
            for (int i = 1; i < MAX_TARGETS; ++i) {
                if (snapshot.is_valid(i)) tracked++;
            }
            stats.update(latency, tracked > 0);
            dashboard::set("predictor.latency_ms", latency);
            dashboard::set("predictor.tracked_count", tracked);
            dashboard::set("predictor.fps", stats.last_fps);

        } catch (const umt::MessageError_Timeout&) {
            // 超时，继续
        } catch (const umt::MessageError_Stopped&) {
            // 没有 publisher，等待 detector 启动
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "PredictorNode",
                "Exception: {}", e.what());
        }
    }

    debug::print(debug::PrintMode::INFO, "PredictorNode", "Predictor node stopped");
}

}  // namespace autoaim::predictor
