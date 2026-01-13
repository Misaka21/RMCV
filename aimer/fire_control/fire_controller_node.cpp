/**
 * @file fire_controller_node.cpp
 * @brief 统一火控节点实现
 *
 * 顶层调度，拥有各模式的 Decider 实例
 */

#include "fire_controller_node.hpp"

#include <chrono>
#include <thread>

#include "aimer/common/robot_state.hpp"
#include "aimer/fire_control/core/types.hpp"
#include "aimer/auto_aim/target_decision/autoaim_decider.hpp"
#include "umt/BasicObjManager.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/watchdog/watchdog_node.hpp"

namespace fire_control {

namespace {

// 获取当前时间 (秒)
double get_current_time() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

}  // namespace

void fire_control_run(const std::string& /* config_path */) {
    debug::print(debug::PrintMode::INFO, "FireControlNode", "Starting fire control node...");

    // 输出指令
    auto fire_cmd = umt::BasicObjManager<::fire_control::FireCommand>::find_or_create("fire_command");
    auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    // 实例化决策器 (具体类型)
    autoaim::target_decision::AutoAimDecider autoaim_decider;

    // TODO: 能量机关决策器
    // energy::target_decision::EnergyDecider energy_decider;

    // 模式跟踪
    aimer::AimMode last_mode = aimer::AimMode::DISABLED;

    debug::print(debug::PrintMode::INFO, "FireControlNode", "Fire control node started, running at 100Hz");

    // 主循环 (100Hz)
    const auto period = std::chrono::microseconds(10000);  // 10ms
    auto next_time = std::chrono::steady_clock::now();

    while (app_running->get()) {
        watchdog::heartbeat("fire_control");

        double current_time = get_current_time();

        // 获取当前模式
        aimer::AimMode mode = autoaim_decider.current_mode();

        // 模式切换检测
        if (mode != last_mode) {
            debug::print(debug::PrintMode::INFO, "FireControlNode",
                "Mode switch: {} -> {}",
                aimer::aim_mode_name(last_mode),
                aimer::aim_mode_name(mode));

            // 重置控制器状态
            if (last_mode == aimer::AimMode::AUTOAIM) {
                autoaim_decider.reset();
            }
            // TODO: 能量机关模式切换时重置
            // if (last_mode == aimer::AimMode::ENERGY_SMALL ||
            //     last_mode == aimer::AimMode::ENERGY_LARGE) {
            //     energy_decider.reset();
            // }
            last_mode = mode;
        }

        // 根据模式调用对应决策器
        FireCommand cmd{};

        switch (mode) {
        case aimer::AimMode::AUTOAIM:
            cmd = autoaim_decider.decide(current_time);
            break;

        case aimer::AimMode::ENERGY_SMALL:
        case aimer::AimMode::ENERGY_LARGE: {
            // 能量机关模式 - 待实现
            // cmd = energy_decider.decide(current_time);
            static bool warned = false;
            if (!warned) {
                debug::print(debug::PrintMode::WARNING, "FireControlNode",
                    "Energy mode not implemented yet");
                warned = true;
            }
            cmd.control_enabled = false;
            break;
        }

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

    debug::print(debug::PrintMode::INFO, "FireControlNode", "Fire control node stopped");
}

void start_fire_control(const std::string& config_path) {
    std::thread([config_path]() {
        fire_control_run(config_path);
    }).detach();
}

}  // namespace fire_control
