/**
 * @file fire_control_node.cpp
 * @brief 自瞄火控节点实现
 */

#include "fire_control_node.hpp"

#include <chrono>
#include <thread>

#include "aimer/common/robot_state.hpp"
#include "fire_controller.hpp"
#include "common/latency_estimator.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/fire_control_types.hpp"
#include "umt/BasicObjManager.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/watchdog/watchdog_node.hpp"

namespace autoaim::fire_control {

namespace {

// 获取当前时间 (秒)
double get_current_time() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

}  // namespace

void fire_control_run(const std::string& /* config_path */) {
    debug::print(debug::PrintMode::INFO, "AutoAimFireControl", "Starting...");

    // 数据源
    auto battlefield = umt::BasicObjManager<predictor::BattlefieldSnapshot>::find_or_create("battlefield");
    auto fire_cmd = umt::BasicObjManager<::fire_control::FireCommand>::find_or_create("fire_command");
    auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    // 自瞄控制器
    FireController controller;

    // 延迟估计器
    LatencyEstimator latency_estimator;

    // 模式跟踪
    aimer::AimMode last_mode = aimer::AimMode::DISABLED;
    int last_frame_id = -1;

    debug::print(debug::PrintMode::INFO, "AutoAimFireControl", "Running at 100Hz");

    // 主循环 (100Hz)
    const auto period = std::chrono::microseconds(10000);  // 10ms
    auto next_time = std::chrono::steady_clock::now();

    while (app_running->get()) {
        watchdog::heartbeat("autoaim_fire_control");

        // 获取战场快照
        const auto& snapshot = battlefield->get();
        double current_time = get_current_time();

        // 获取当前模式
        aimer::AimMode mode = snapshot.self_state.aim_mode;

        // 检测新帧，更新延迟估计
        if (snapshot.frame_id != last_frame_id && snapshot.predict_timestamp > 0) {
            last_frame_id = snapshot.frame_id;
            double predict_to_send = current_time - snapshot.predict_timestamp;
            latency_estimator.update_predict_to_send(predict_to_send, current_time);
        }

        // 模式切换检测
        if (mode != last_mode) {
            debug::print(debug::PrintMode::INFO, "AutoAimFireControl",
                "Mode switch: {} -> {}",
                aimer::aim_mode_name(last_mode),
                aimer::aim_mode_name(mode));

            // 重置控制器状态
            if (mode == aimer::AimMode::AUTOAIM) {
                controller.reset();
            }
            last_mode = mode;
        }

        // 构建延迟信息
        LatencyInfo latency = latency_estimator.build(snapshot, current_time);

        // 根据模式处理
        ::fire_control::FireCommand cmd{};

        switch (mode) {
        case aimer::AimMode::AUTOAIM:
            cmd = controller.control(snapshot, current_time, latency);
            break;

        case aimer::AimMode::ENERGY_SMALL:
        case aimer::AimMode::ENERGY_LARGE:
            // 能量机关模式 - 由 autobuff 模块处理
            cmd.control_enabled = false;
            break;

        case aimer::AimMode::DISABLED:
        default:
            cmd.control_enabled = false;
            break;
        }

        // 输出控制指令
        fire_cmd->get() = cmd;

        // 等待下一周期
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }

    debug::print(debug::PrintMode::INFO, "AutoAimFireControl", "Stopped");
}

void start_fire_control_node(const std::string& config_path) {
    std::thread([config_path]() {
        fire_control_run(config_path);
    }).detach();
}

}  // namespace autoaim::fire_control
