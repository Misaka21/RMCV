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
            // 注: 这里假设 "发送" 发生在火控收到数据之前
            // 实际上应该在串口发送后更新，这里用 predict→fire_control 作为近似
            double predict_to_send = current_time - snapshot.predict_timestamp;
            latency_estimator.update_predict_to_send(predict_to_send, current_time);

            // 调试: 打印延迟信息
            double img_to_predict = snapshot.predict_timestamp - snapshot.timestamp;
            // debug::print("debug", "FireControl",
            //     "Latency: img_to_predict={:.1f}ms, predict_to_send={:.1f}ms",
            //     img_to_predict * 1000, latency_estimator.get_predict_to_send() * 1000
            // );
        }

        // 执行控制 (参数在内部实时读取)
        FireCommand cmd = controller.control(snapshot, current_time);

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
