/**
 * @file autoaim_decider.hpp
 * @brief 自瞄目标决策器
 *
 * 封装自瞄的目标选择和开火决策逻辑
 *
 * 职责:
 *   1. 从 UMT 读取 BattlefieldSnapshot (内部细节，对外不暴露)
 *   2. 目标选择、弹道解算、开火判断
 *   3. 输出 FireCommand
 *
 * 数据流:
 *   FireControllerNode 调用 decide()
 *       ↓
 *   AutoAimDecider 内部从 UMT 读取 BattlefieldSnapshot
 *       ↓
 *   FireController::control() (复用现有逻辑)
 *       ↓
 *   FireCommand
 */

#ifndef __AIMER_AUTO_AIM_TARGET_DECISION_AUTOAIM_DECIDER_HPP__
#define __AIMER_AUTO_AIM_TARGET_DECISION_AUTOAIM_DECIDER_HPP__

#include "aimer/common/robot_state.hpp"
#include "aimer/fire_control/core/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "fire_controller.hpp"
#include "common/latency_estimator.hpp"
#include "umt/BasicObjManager.hpp"

namespace autoaim::target_decision {

/**
 * @brief 自瞄目标决策器
 *
 * 内部封装 BattlefieldSnapshot 的访问
 * 对外只暴露 decide() 和 FireCommand 输出
 */
class AutoAimDecider {
public:
    AutoAimDecider();
    ~AutoAimDecider() = default;

    // 禁止拷贝
    AutoAimDecider(const AutoAimDecider&) = delete;
    AutoAimDecider& operator=(const AutoAimDecider&) = delete;

    /**
     * @brief 执行决策
     *
     * @param current_time 当前时间 (秒)
     * @return FireCommand 火控指令
     *
     * @note 内部从 BattlefieldSnapshot 获取数据
     */
    ::fire_control::FireCommand decide(double current_time);

    /**
     * @brief 获取当前模式
     *
     * 从 BattlefieldSnapshot.self_state 读取 aim_mode
     */
    aimer::AimMode current_mode() const;

    /**
     * @brief 重置状态
     */
    void reset();

private:
    // 从 UMT 读取 Predictor 输出 (内部细节)
    umt::BasicObjManager<predictor::BattlefieldSnapshot>::sptr battlefield_;

    // 复用现有的火控逻辑
    fire_control::FireController fire_controller_;

    // 延迟估计器
    fire_control::LatencyEstimator latency_estimator_;

    // 帧 ID 跟踪
    int last_frame_id_ = -1;
};

}  // namespace autoaim::target_decision

#endif  // __AIMER_AUTO_AIM_TARGET_DECISION_AUTOAIM_DECIDER_HPP__
