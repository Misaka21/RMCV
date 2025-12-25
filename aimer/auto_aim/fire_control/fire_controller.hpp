/**
 * @file fire_controller.hpp
 * @brief 火控调度器
 *
 * 职责:
 *   1. 根据配置选择控制模式 (MPC 或 PID)
 *   2. 分发处理请求到对应策略
 *   3. 统一对外接口
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__

#include <memory>

#include "aimer/auto_aim/fire_control/types.hpp"
#include "aimer/auto_aim/fire_control/fire_strategy.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::fire_control {

/**
 * @brief 控制模式
 */
enum class ControlMode {
    MPC,    // MPC 轨迹规划模式 (输出 pos+vel+acc)
    PID     // PID 跟踪模式 (仅输出 pos，含反陀螺逻辑)
};

/**
 * @brief 火控调度器
 *
 * 根据配置选择 MPC 或 PID 模式，分发处理请求
 * 参数通过 runtime_param::get_param 实时获取
 */
class FireController {
public:
    FireController();
    ~FireController();

    // 禁止拷贝
    FireController(const FireController&) = delete;
    FireController& operator=(const FireController&) = delete;

    /**
     * @brief 主控制函数
     *
     * @param snapshot 战场快照 (来自 Predictor)
     * @param current_time 当前时间 (s)
     * @return 火控指令
     */
    FireCommand control(
        const predictor::BattlefieldSnapshot& snapshot,
        double current_time
    );

    /**
     * @brief 重置状态
     */
    void reset();

    /**
     * @brief 获取当前控制模式
     */
    ControlMode current_mode() const { return current_mode_; }

    /**
     * @brief 获取当前策略名称
     */
    const char* strategy_name() const;

    // ==================== 调试接口 ====================

    const TargetSelection& last_selection() const;
    const AimResult& last_aim() const;

    /**
     * @brief 获取当前策略 (用于高级调试)
     */
    FireStrategy* current_strategy() const { return current_strategy_; }

private:
    /**
     * @brief 确保策略已初始化并更新模式
     */
    void ensure_strategy_initialized();

    std::unique_ptr<FireStrategy> mpc_strategy_;
    std::unique_ptr<FireStrategy> pid_strategy_;

    FireStrategy* current_strategy_ = nullptr;
    ControlMode current_mode_ = ControlMode::PID;  // 默认 PID 模式
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_FIRE_CONTROLLER_HPP__
