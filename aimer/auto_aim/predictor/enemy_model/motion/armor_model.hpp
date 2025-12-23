/**
 * @file armor_model.hpp
 * @brief 装甲板运动模型 - YPD坐标系 EKF 滤波
 *
 * 参考: rm.cv.fans 的 ArmorModel
 *
 * 改进:
 * - 使用YPD坐标系滤波 (更适合旋转目标)
 * - Ceres自动微分计算雅可比 (无需手动推导)
 *
 * 状态向量: [yaw, vyaw, pitch, vpitch, dis, vdis]
 * 观测向量: [yaw, pitch, dis]
 *
 * 职责 (单一):
 * - 对每个装甲板 ID 独立 EKF 滤波
 * - 输出滤波后的位置和速度
 *
 * 不做:
 * - ID 分配 (由 ArmorIdentifier 完成)
 * - 整车建模 (由 VehicleEkf 完成)
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_MOTION_ARMOR_MODEL_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_MOTION_ARMOR_MODEL_HPP__

#include <map>
#include <vector>

#include <Eigen/Core>

#include "aimer/auto_aim/predictor/enemy_state/armor_identifier.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/filter/position_ekf_ypd.hpp"

namespace autoaim::predictor {

/**
 * @brief 单个装甲板滤波线程
 *
 * 匀速模型 (CV): 状态 [yaw, vyaw, pitch, vpitch, dis, vdis]
 * 使用Ceres自动微分EKF
 */
class FilterThread {
public:
    /**
     * @brief 构造
     * @param armor 初始装甲板数据
     * @param timestamp 时间戳
     * @param credit_time 超时时间 (s)
     */
    FilterThread(const ArmorData& armor, double timestamp, double credit_time);

    /**
     * @brief 更新滤波器
     * @param armor 新的装甲板数据
     * @param timestamp 时间戳
     */
    void update(const ArmorData& armor, double timestamp);

    /**
     * @brief 是否有效 (未超时)
     */
    bool credit(double current_time) const;

    /**
     * @brief 预测位置 (XYZ)
     */
    Eigen::Vector3d predict_pos(double timestamp) const;

    /**
     * @brief 预测速度 (XYZ)
     */
    Eigen::Vector3d predict_vel(double timestamp) const;

    /**
     * @brief 预测YPD坐标
     */
    math::YpdCoord predict_ypd(double timestamp) const;

    /**
     * @brief 预测YPD速度
     */
    math::YpdCoord predict_ypd_v(double timestamp) const;

    /**
     * @brief 获取装甲板状态
     */
    ArmorState get_armor_state(double timestamp) const;

    // Getters
    int id() const { return armor_.id; }
    const ArmorData& armor() const { return armor_; }
    double last_update() const { return last_update_time_; }

private:
    ArmorData armor_;                // 最近的装甲板数据
    double last_update_time_ = 0;    // 最后更新时间
    double credit_time_;             // 超时阈值

    // YPD坐标系EKF滤波器 (自动微分)
    filter::PositionEkfYpd ekf_;
};

/**
 * @brief 装甲板运动模型
 *
 * 管理多个 FilterThread，负责:
 * - 按 ID 匹配滤波器
 * - 超时清理
 * - 输出滤波后的装甲板状态
 */
class ArmorModel {
public:
    /**
     * @brief 构造
     * @param credit_time 滤波器超时时间 (s)
     */
    explicit ArmorModel(double credit_time = 0.1);

    /**
     * @brief 更新所有滤波器
     * @param armors 带 ID 的装甲板数据 (来自 ArmorIdentifier)
     * @param timestamp 时间戳
     */
    void update(const std::vector<ArmorData>& armors, double timestamp);

    /**
     * @brief 获取所有滤波后的装甲板状态
     */
    std::vector<ArmorState> get_armor_states(double timestamp) const;

    /**
     * @brief 获取最佳装甲板
     * @return 最佳装甲板指针，无则 nullptr
     */
    const FilterThread* get_best(double timestamp) const;

    /**
     * @brief 滤波器数量
     */
    size_t size() const { return filters_.size(); }

    /**
     * @brief 重置
     */
    void reset();

private:
    std::map<int, FilterThread> filters_;  // armor_id -> filter
    double credit_time_;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_MOTION_ARMOR_MODEL_HPP__
