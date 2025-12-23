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
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_HPP__

#include <array>
#include <memory>

#include <opencv2/core.hpp>

#include "aimer/auto_aim/common/types.hpp"
#include "aimer/auto_aim/predictor/enemy_state/armor_observer.hpp"
#include "aimer/auto_aim/predictor/target_catcher.hpp"
#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::predictor {

// 前向声明
class EnemyModelInterface;
class EnemyModelFactory;

/**
 * @brief 敌方预测器
 */
class EnemyPredictor {
public:
    EnemyPredictor();
    ~EnemyPredictor();

    /**
     * @brief 主预测函数
     * @param detection 检测结果
     * @return 战场快照
     */
    BattlefieldSnapshot predict(const autoaim::DetectionResult& detection);

    /**
     * @brief 设置相机内参
     */
    void set_camera_params(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);

    // 调试用
    const ArmorObservationTable& get_observation_table() const { return observer_.table(); }
    int get_target() const { return target_catcher_.get_target(current_time_); }

private:
    /**
     * @brief 阶段1: 观测 (PnP 解算)
     */
    void update_observations(const autoaim::DetectionResult& detection);

    /**
     * @brief 阶段2: 更新模型 (消抖 + EKF)
     */
    void update_models();

    /**
     * @brief 选择目标
     */
    void select_target();

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

    // 目标选择器
    TargetCatcher target_catcher_;

    // 时间
    double current_time_ = 0;
    int frame_id_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_HPP__
