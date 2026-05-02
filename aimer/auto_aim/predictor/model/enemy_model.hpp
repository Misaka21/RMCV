/**
 * @file enemy_model.hpp
 * @brief 运动模型接口和工厂
 *
 * 模型类型:
 * - VEHICLE: 车辆 (4块装甲板) - 英雄/工程/步兵/哨兵
 * - OUTPOST: 前哨站 (3块小装甲板)
 * - BASE: 基地 (固定建筑)
 *
 * 每个模型负责:
 * - 消抖过滤
 * - 装甲板 ID 分配
 * - EKF 滤波
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_ENEMY_MODEL_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_ENEMY_MODEL_HPP__

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::predictor {

// ============================================================================
// 运动模型接口
// ============================================================================

class EnemyModelInterface {
public:
    virtual ~EnemyModelInterface() = default;

    /**
     * @brief 更新模型
     * @param observations 该目标的装甲板观测列表
     * @param timestamp 时间戳
     */
    virtual void update(const std::vector<ArmorObservation>& observations, double timestamp) = 0;

    /**
     * @brief 预测状态
     * @param timestamp 预测时间点
     */
    virtual TargetState predict(double timestamp) const = 0;

    /**
     * @brief 是否存活 (有效)
     */
    virtual bool alive() const = 0;

    /**
     * @brief 重置
     */
    virtual void reset() = 0;

    /**
     * @brief 获取目标 ID
     */
    virtual int target_id() const = 0;

    /**
     * @brief 获取模型类型名
     */
    virtual const char* type_name() const = 0;

    /**
     * @brief 绘制调试信息
     * @param img 输出图像
     * @param q_imu IMU四元数 (用于世界坐标投影到图像)
     * @param timestamp 当前时间戳
     *
     * 绘制内容:
     * - 检测到的装甲板 (绿色)
     * - 滤波/预测的装甲板 (蓝色)
     * - 旋转中心 (陀螺模式, 红色)
     */
    virtual void draw(cv::Mat& img, const Eigen::Quaterniond& q_imu, double timestamp) const {
        // 默认空实现
        (void)img; (void)q_imu; (void)timestamp;
    }
};

// ============================================================================
// 模型类型
// ============================================================================

enum class ModelType {
    VEHICLE,   // 车辆 (4装甲板): 英雄/工程/步兵/哨兵
    OUTPOST,   // 前哨站 (3装甲板)
    BASE       // 基地 (固定)
};

// ============================================================================
// EnemyType → ModelType 映射
// ============================================================================

inline const std::unordered_map<EnemyType, ModelType> ENEMY_TO_MODEL = {
    { EnemyType::UNKNOWN,    ModelType::VEHICLE },
    { EnemyType::HERO,       ModelType::VEHICLE },
    { EnemyType::ENGINEER,   ModelType::VEHICLE },
    { EnemyType::INFANTRY_3, ModelType::VEHICLE },
    { EnemyType::INFANTRY_4, ModelType::VEHICLE },
    { EnemyType::INFANTRY_5, ModelType::VEHICLE },
    { EnemyType::SENTRY,     ModelType::VEHICLE },
    { EnemyType::OUTPOST,    ModelType::OUTPOST },
    { EnemyType::BASE,       ModelType::BASE },
};

inline ModelType get_model_type(EnemyType enemy_type) {
    auto it = ENEMY_TO_MODEL.find(enemy_type);
    return (it != ENEMY_TO_MODEL.end()) ? it->second : ModelType::VEHICLE;
}

// ============================================================================
// 模型工厂
// ============================================================================

// 模型构造函数类型
using ModelCreator = std::function<
    std::unique_ptr<EnemyModelInterface>(int target_id, EnemyType enemy_type)
>;

/**
 * @brief 运动模型工厂
 */
class EnemyModelFactory {
public:
    EnemyModelFactory();

    /**
     * @brief 创建模型
     * @param target_id 目标 ID
     * @param enemy_type 目标类型
     */
    std::unique_ptr<EnemyModelInterface> create(int target_id, EnemyType enemy_type);

private:
    std::unordered_map<ModelType, ModelCreator> model_map_;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_ENEMY_MODEL_HPP__
