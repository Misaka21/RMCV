/**
 * @file enemy_predictor.hpp
 * @brief 敌方预测器主类
 *
 * 数据流:
 *   DetectionResult
 *        ↓ ArmorObserver [PnP]
 *   ArmorObservationTable
 *        ↓ EnemyModel [消抖 + EKF]
 *   BattlefieldSnapshot → 火控
 *
 * 职责: 预测所有敌人状态，不做目标选择 (由火控负责)
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_HPP__

#include <array>
#include <memory>

#include <opencv2/core.hpp>

#include "aimer/common/types.hpp"
#include "aimer/auto_aim/predictor/enemy_state/armor_observer.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::predictor {

// 前向声明
class EnemyModelInterface;
class EnemyModelFactory;

/**
 * @brief 敌方预测器
 *
 * 负责预测所有敌方车辆状态，输出 BattlefieldSnapshot 给火控
 */
class EnemyPredictor {
public:
    EnemyPredictor();
    ~EnemyPredictor();

    /**
     * @brief 主预测函数
     * @param detection 检测结果
     * @param timestamp 当前时间戳 (s)
     * @return 战场快照 (所有敌人状态)
     */
    BattlefieldSnapshot predict(const DetectionResult& detection, double timestamp);

    // 调试用
    const ArmorObservationTable& get_observation_table() const { return observer_.table(); }

private:
    /**
     * @brief 阶段1: 观测 (PnP 解算)
     */
    void update_observations(const DetectionResult& detection, double timestamp);

    /**
     * @brief 阶段2: 更新模型 (消抖 + EKF)
     */
    void update_models();

    /**
     * @brief 导出战场快照
     */
    BattlefieldSnapshot export_snapshot();

    // ==================== 数据 ====================

    // 装甲板观测器
    ArmorObserver observer_;

    // 各目标运动模型 (索引 = target_id)
    std::array<std::unique_ptr<EnemyModelInterface>, MAX_TARGETS> enemy_models_;

    // 模型工厂
    std::unique_ptr<EnemyModelFactory> model_factory_;

    // 时间
    double current_time_ = 0;
    int frame_id_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_HPP__
