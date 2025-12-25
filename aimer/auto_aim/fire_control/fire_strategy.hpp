/**
 * @file fire_strategy.hpp
 * @brief 火控策略接口
 *
 * 两种策略:
 * - MPC: 使用 TinyMPC 规划轨迹，输出 pos+vel+acc
 * - PID: 直接跟踪目标位置，含反陀螺逻辑
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_STRATEGY_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_STRATEGY_HPP__

#include "types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

/**
 * @brief 火控策略接口
 */
class FireStrategy {
public:
    virtual ~FireStrategy() = default;

    /**
     * @brief 处理一帧数据，生成火控指令
     *
     * @param snapshot 战场快照
     * @param current_time 当前时间 (s)
     * @return 火控指令
     */
    virtual FireCommand process(
        const predictor::BattlefieldSnapshot& snapshot,
        double current_time
    ) = 0;

    /**
     * @brief 重置状态
     */
    virtual void reset() = 0;

    /**
     * @brief 获取策略名称 (调试用)
     */
    virtual const char* name() const = 0;

    // ==================== 调试接口 ====================

    virtual const TargetSelection& last_selection() const = 0;
    virtual const AimResult& last_aim() const = 0;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_STRATEGY_HPP__
