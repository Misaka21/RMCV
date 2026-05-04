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

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "aimer/common/types.hpp"
#include "aimer/auto_aim/predictor/observer/armor_observer.hpp"
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

    /**
     * @brief 绘制所有模型的调试信息
     * @param img 输出图像
     * @param q_imu IMU 四元数
     * @param timestamp 当前时间戳
     */
    void draw(cv::Mat& img, const Eigen::Quaterniond& q_imu, double timestamp) const;

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

    /**
     * @brief 低频输出指定目标的 predictor 诊断信息
     */
    void log_debug_snapshot(const BattlefieldSnapshot& snapshot) const;

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

    // 当前帧自身状态
    aimer::RobotState current_state_;

    // 模型最近一次被真实观测更新的时间 (用于 observer 纠正时过滤过期模型)
    std::array<double, MAX_TARGETS> model_last_seen_time_ = {};

    // 新目标消抖状态：需连续出现一段时间才创建模型
    std::array<double, MAX_TARGETS> pending_first_seen_time_ = {};
    std::array<double, MAX_TARGETS> pending_last_seen_time_ = {};

    // 指定目标调试日志状态
    mutable double last_debug_log_time_ = -1.0;
    mutable int last_debug_target_id_ = -1;
    mutable bool last_debug_valid_ = false;
    mutable bool last_debug_detected_ = false;
    mutable int last_debug_spin_level_ = -1;
    mutable int last_debug_recommended_idx_ = -2;
    mutable uint8_t last_debug_visible_mask_ = 0;
    mutable size_t last_debug_obs_count_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_HPP__
