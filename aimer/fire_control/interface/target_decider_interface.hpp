/**
 * @file target_decider_interface.hpp
 * @brief 目标决策接口 - 抽象的决策器基类
 *
 * 设计目的:
 *   1. 顶层火控通过此接口调用决策器，不知道具体实现
 *   2. AutoAim 和 Energy 模块各自实现此接口
 *   3. 实现封装各自的数据源 (BattlefieldSnapshot / EnergySnapshot)
 *
 * 数据流:
 *   FireControllerNode (拥有 Decider 实例)
 *       ↓ 调用 decide()
 *   TargetDeciderInterface::decide()
 *       ↓ 内部访问数据源 (Decider 实现知道具体类型)
 *   AimingDecision (抽象输出)
 */

#ifndef __AIMER_FIRE_CONTROL_INTERFACE_TARGET_DECIDER_INTERFACE_HPP__
#define __AIMER_FIRE_CONTROL_INTERFACE_TARGET_DECIDER_INTERFACE_HPP__

#include "aimer/common/aiming_decision.hpp"
#include "aimer/common/robot_state.hpp"  // AimMode

namespace fire_control {

/**
 * @brief 目标决策接口
 *
 * 实现类负责:
 *   1. 从各自的数据源获取目标信息和自身状态 (内部细节)
 *   2. 目标选择
 *   3. 弹道解算
 *   4. 开火判断
 *   5. 输出 AimingDecision
 */
class TargetDeciderInterface {
public:
    virtual ~TargetDeciderInterface() = default;

    /**
     * @brief 执行决策
     *
     * @param current_time 当前时间 (秒, steady_clock)
     * @return AimingDecision 瞄准决策结果
     *
     * @note 实现类内部从 UMT 读取数据源 (BattlefieldSnapshot 等)
     *       调用者不需要知道数据源的具体类型
     *       自身状态也由实现类内部获取
     */
    virtual aimer::AimingDecision decide(double current_time) = 0;

    /**
     * @brief 重置状态
     *
     * 在模式切换时调用，清空内部跟踪状态
     */
    virtual void reset() = 0;

    /**
     * @brief 获取当前模式
     *
     * 顶层火控需要知道当前应该使用哪个模式
     * @return AimMode 当前模式 (由实现类从数据源获取)
     */
    virtual aimer::AimMode current_mode() const = 0;

    /**
     * @brief 获取决策器名称 (用于日志)
     */
    virtual const char* name() const = 0;
};

}  // namespace fire_control

#endif  // __AIMER_FIRE_CONTROL_INTERFACE_TARGET_DECIDER_INTERFACE_HPP__
