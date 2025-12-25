/**
 * @file fire_controller_node.cpp
 * @brief 火控线程节点实现
 *
 * 延迟估计在此处进行:
 *   img_to_predict = predict_timestamp - timestamp  (直接计算)
 *   predict_to_send = now - predict_timestamp       (卡尔曼滤波)
 */

#include "fire_controller_node.hpp"

#include <chrono>
#include <thread>

#include "fire_controller.hpp"
#include "common/latency_estimator.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "umt/BasicObjManager.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

namespace {

// 获取当前时间 (秒)
double get_current_time()
{
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

}  // namespace

void fire_control_run(const std::string& /* config_path */)
{
    debug::print("info", "FireControl", "Starting fire control node...");

    // 创建火控器 (参数在内部直接通过 get_param 获取)
    FireController controller;

    // 延迟估计器
    LatencyEstimator latency_estimator;

    // 获取共享数据
    auto battlefield = umt::BasicObjManager<predictor::BattlefieldSnapshot>::find_or_create("battlefield");
    auto fire_cmd = umt::BasicObjManager<FireCommand>::find_or_create("fire_command");

    debug::print("info", "FireControl", "Fire control node started, running at 100Hz");

    // 上一帧信息 (用于检测新帧)
    int last_frame_id = -1;

    // 主循环 (100Hz)
    const auto period = std::chrono::microseconds(10000);  // 10ms
    auto next_time = std::chrono::steady_clock::now();

    while (true) {
        // 获取战场快照
        const auto& snapshot = battlefield->get();

        // 获取当前时间
        double current_time = get_current_time();

        // 检测新帧，更新延迟估计
        if (snapshot.frame_id != last_frame_id && snapshot.predict_timestamp > 0) {
            last_frame_id = snapshot.frame_id;

            // 计算 predict_to_send 延迟 (本次观测)
            double predict_to_send = current_time - snapshot.predict_timestamp;
            latency_estimator.update_predict_to_send(predict_to_send, current_time);
        }

        // ========== 构建延迟信息 ==========
        LatencyInfo latency;

        // img_to_predict: 直接计算
        latency.img_to_predict = (snapshot.predict_timestamp > 0)
            ? (snapshot.predict_timestamp - snapshot.timestamp)
            : 0.015;  // 默认 15ms

        // predict_to_send: 卡尔曼滤波
        latency.predict_to_send = latency_estimator.get_predict_to_send();

        // send_to_control: 静态配置
        latency.send_to_control = runtime_param::get_param<double>(
            "AutoAim.FireControl.Latency.send_to_control"
        );

        // control_to_fire: 静态配置
        latency.control_to_fire = runtime_param::get_param<double>(
            "AutoAim.FireControl.Latency.control_to_fire"
        );

        // fire_to_hit: 根据目标距离计算
        // 这里使用主目标的距离，如果没有目标则使用默认值
        double distance = 5.0;  // 默认 5m
        if (snapshot.get_primary()) {
            const auto* armor = snapshot.get_primary()->get_recommended_armor();
            if (armor) {
                distance = armor->position.norm();
            }
        }
        double bullet_speed = std::max(snapshot.self_state.bullet_speed, 10.0);
        latency.fire_to_hit = distance / bullet_speed;

        // ========== 执行控制 ==========
        FireCommand cmd = controller.control(snapshot, current_time, latency);

        // 输出控制指令
        fire_cmd->get() = cmd;

        // 等待下一周期
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}

void start_fire_control(const std::string& config_path)
{
    std::thread([config_path]() {
        fire_control_run(config_path);
    }).detach();
}

void background_fire_control_run(const std::string& config_path)
{
    start_fire_control(config_path);
}

}  // namespace autoaim::fire_control
