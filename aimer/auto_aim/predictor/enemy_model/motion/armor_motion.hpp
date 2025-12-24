/**
 * @file armor_motion.hpp
 * @brief 装甲板运动模型 - YPD坐标系 EKF 滤波
 *
 * 参考: rm.cv.fans 的 ArmorMotion
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

#ifndef __AIMER_AUTO_AIM_PREDICTOR_MOTION_ARMOR_MOTION_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_MOTION_ARMOR_MOTION_HPP__

#include <map>
#include <vector>

#include <Eigen/Core>

#include "aimer/auto_aim/predictor/enemy_state/armor_identifier.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/filter/adaptive_ekf.hpp"
#include "aimer/common/math/math.hpp"

namespace autoaim::predictor {

// ============================================================================
// EKF 预测/观测函数
// ============================================================================

/**
 * @brief 匀速预测函数 (CV模型)
 *
 * 状态: [yaw, vyaw, pitch, vpitch, dis, vdis]
 * 转移: pos' = pos + vel * dt, vel' = vel
 */
struct YpdCVPredict {
    double dt;

    explicit YpdCVPredict(double delta_t) : dt(delta_t) {}

    template<typename T>
    void operator()(const T x_in[6], T x_out[6]) const {
        x_out[0] = x_in[0] + T(dt) * x_in[1];  // yaw
        x_out[1] = x_in[1];
        x_out[2] = x_in[2] + T(dt) * x_in[3];  // pitch
        x_out[3] = x_in[3];
        x_out[4] = x_in[4] + T(dt) * x_in[5];  // dis
        x_out[5] = x_in[5];
    }
};

/**
 * @brief 直接观测函数 (YPD → YPD)
 */
struct YpdDirectMeasure {
    template<typename T>
    void operator()(const T x[6], T y[3]) const {
        y[0] = x[0];  // yaw
        y[1] = x[2];  // pitch
        y[2] = x[4];  // dis
    }
};

// ============================================================================
// 角度处理工具 (已移到 math/math.hpp)
// ============================================================================

// 使用 math::normalize_angle()
// 使用 math::get_closest_angle()

// ============================================================================
// FilterThread - 单个装甲板滤波器
// ============================================================================

/**
 * @brief 单个装甲板滤波线程
 *
 * 匀速模型 (CV): 状态 [yaw, vyaw, pitch, vpitch, dis, vdis]
 * 使用Ceres自动微分EKF
 */
class FilterThread {
public:
    using Ekf = filter::AdaptiveEkf<6, 3>;
    using VectorX = Eigen::Matrix<double, 6, 1>;
    using VectorY = Eigen::Matrix<double, 3, 1>;
    using MatrixXX = Eigen::Matrix<double, 6, 6>;
    using MatrixYY = Eigen::Matrix<double, 3, 3>;

    /**
     * @brief 构造
     * @param armor 初始装甲板数据
     * @param timestamp 时间戳
     * @param credit_time 超时时间 (s)
     */
    FilterThread(const ArmorData& armor, double timestamp, double credit_time);

    /**
     * @brief 更新滤波器
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

    // EKF 滤波器
    Ekf ekf_;
};

// ============================================================================
// ArmorMotion - 装甲板运动模型
// ============================================================================

/**
 * @brief 装甲板运动模型
 *
 * 管理多个 FilterThread，负责:
 * - 按 ID 匹配滤波器
 * - 超时清理
 * - 输出滤波后的装甲板状态
 */
class ArmorMotion {
public:
    /**
     * @brief 构造
     * @param credit_time 滤波器超时时间 (s)
     */
    explicit ArmorMotion(double credit_time = 0.1);

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

#endif  // __AIMER_AUTO_AIM_PREDICTOR_MOTION_ARMOR_MOTION_HPP__
