/**
 * @file outpost_motion.hpp
 * @brief 前哨站运动模型 (EKF)
 *
 * 前哨站特点 (2025规则):
 * - 3块装甲板，相隔120度
 * - 匀速旋转，|ω| = 0.8π rad/s (方向随机)
 * - 固定半径 r = 0.2765m (直径 0.553m)
 * - 三块装甲板高度不同 (低/中/高，相隔10cm)
 *
 * 状态向量 (7维):
 *   [xc, vx, yc, vy, zc, θ, ω]
 *   - xc, yc, zc: 旋转中心位置 (世界系)
 *   - vx, vy: 中心速度 (前哨站可能在移动平台上?)
 *   - θ: 相位 (rad)
 *   - ω: 角速度 (rad/s)，约束 |ω| ≈ 0.8π
 *
 * 外部维护:
 *   - 每个 armor_id 对应的高度差 dz
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_MOTION_OUTPOST_MOTION_MODEL_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_MOTION_OUTPOST_MOTION_MODEL_HPP__

#include <array>
#include <cmath>

#include <Eigen/Core>
#include <ceres/jet.h>

#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/auto_aim/predictor/enemy_state/armor_identifier.hpp"
#include "aimer/common/filter/adaptive_ekf.hpp"

namespace autoaim::predictor {

// ============================================================================
// 前哨站常量 (规则固定)
// ============================================================================

namespace outpost_model {

// 规则参数
constexpr double OMEGA_ABS = 0.8 * M_PI;  // 角速度绝对值 (rad/s)
constexpr double RADIUS = 0.2765;          // 旋转半径 (m), 直径 0.553m
constexpr double DZ_STEP = 0.10;           // 高度差间隔 (m)

// 顶部装甲板过滤 (pitch 阈值)
constexpr double TOP_ARMOR_PITCH_THRESHOLD = 45.0 * M_PI / 180.0;  // 45°

// 状态维度
constexpr int N_X = 7;
constexpr int N_Z = 4;

// 状态索引
enum StateIdx {
    XC = 0,     // 旋转中心 X
    VX = 1,     // X 速度
    YC = 2,     // 旋转中心 Y
    VY = 3,     // Y 速度
    ZC = 4,     // 旋转中心 Z
    THETA = 5,  // 相位
    OMEGA = 6   // 角速度 (约束 |ω| ≈ 0.8π)
};

// 观测索引 (和 SpinMotion 一样)
enum ObsIdx {
    YAW = 0,
    PITCH = 1,
    DIS = 2,
    ARMOR_YAW = 3
};

}  // namespace outpost_motion

// ============================================================================
// 预测函数
// ============================================================================

/**
 * @brief 匀速预测 (和 SpinMotion 类似，但没有 r 状态)
 */
struct OutpostCVPredict {
    double dt;

    explicit OutpostCVPredict(double delta_t) : dt(delta_t) {}

    template<typename T>
    void operator()(const T x_in[outpost_model::N_X], T x_out[outpost_model::N_X]) const {
        x_out[outpost_model::XC] = x_in[outpost_model::XC] + T(dt) * x_in[outpost_model::VX];
        x_out[outpost_model::VX] = x_in[outpost_model::VX];

        x_out[outpost_model::YC] = x_in[outpost_model::YC] + T(dt) * x_in[outpost_model::VY];
        x_out[outpost_model::VY] = x_in[outpost_model::VY];

        x_out[outpost_model::ZC] = x_in[outpost_model::ZC];  // 前哨站高度固定

        x_out[outpost_model::THETA] = x_in[outpost_model::THETA] + T(dt) * x_in[outpost_model::OMEGA];
        x_out[outpost_model::OMEGA] = x_in[outpost_model::OMEGA];
    }
};

// ============================================================================
// 观测函数
// ============================================================================

/**
 * @brief 状态 → 观测 (YPD + armor_yaw)
 *
 * 装甲板位置:
 *   xa = xc - r·cos(θ)
 *   ya = yc - r·sin(θ)
 *   za = zc + dz
 */
struct OutpostMeasure {
    double dz;  // 当前装甲板高度差

    explicit OutpostMeasure(double height_diff = 0) : dz(height_diff) {}

    template<typename T>
    void operator()(const T x[outpost_model::N_X], T y[outpost_model::N_Z]) const {
        T xc = x[outpost_model::XC];
        T yc = x[outpost_model::YC];
        T zc = x[outpost_model::ZC];
        T theta = x[outpost_model::THETA];

        // 固定半径
        T r = T(outpost_model::RADIUS);

        // 装甲板位置
        T xa = xc - r * ceres::cos(theta);
        T ya = yc - r * ceres::sin(theta);
        T za = zc + T(dz);

        // YPD 观测
        T rho_sq = xa * xa + ya * ya;
        T rho = ceres::sqrt(rho_sq);
        T d = ceres::sqrt(rho_sq + za * za);

        y[outpost_model::YAW] = ceres::atan2(ya, xa);
        y[outpost_model::PITCH] = ceres::atan2(za, rho);
        y[outpost_model::DIS] = d;
        y[outpost_model::ARMOR_YAW] = theta;
    }
};

// ============================================================================
// OutpostMotion
// ============================================================================

/**
 * @brief 前哨站运动模型 (EKF)
 *
 * 注意: 这是底层 EKF 模型，不实现 EnemyModelInterface。
 * 由 enemy_model/OutpostModel 包装使用。
 */
class OutpostMotion {
public:
    using Ekf = aimer::filter::AdaptiveEkf<outpost_model::N_X, outpost_model::N_Z>;
    using VectorX = Eigen::Matrix<double, outpost_model::N_X, 1>;
    using VectorZ = Eigen::Matrix<double, outpost_model::N_Z, 1>;
    using MatrixXX = Eigen::Matrix<double, outpost_model::N_X, outpost_model::N_X>;
    using MatrixZZ = Eigen::Matrix<double, outpost_model::N_Z, outpost_model::N_Z>;

    OutpostMotion();

    /**
     * @brief 初始化
     */
    void init(const ArmorData& armor, double timestamp);

    /**
     * @brief 更新
     */
    void update(const ArmorData& armor, double timestamp);

    /**
     * @brief 预测旋转中心
     */
    Eigen::Vector3d predict_center(double dt) const;

    /**
     * @brief 预测指定槽位的装甲板位置
     * @param slot 槽位 (0, 1, 2)
     * @param dt 预测时间
     */
    Eigen::Vector3d predict_armor_pos(int slot, double dt) const;

    /**
     * @brief 获取当前追踪装甲板位置
     */
    Eigen::Vector3d get_armor_pos() const;

    /**
     * @brief 获取中心速度
     */
    Eigen::Vector3d get_velocity() const;

    /**
     * @brief 获取角速度
     */
    double get_omega() const { return ekf_.get_x()[outpost_model::OMEGA]; }

    /**
     * @brief 获取相位
     */
    double get_theta() const { return ekf_.get_x()[outpost_model::THETA]; }

    /**
     * @brief 获取陀螺等级 (前哨站始终是 HIGH)
     */
    SpinLevel get_spin_level() const { return SpinLevel::HIGH; }

    /**
     * @brief 是否有效
     */
    bool valid() const { return initialized_; }

    /**
     * @brief 重置
     */
    void reset();

    /**
     * @brief 获取状态向量
     */
    VectorX get_state() const { return ekf_.get_x(); }

    /**
     * @brief 获取当前追踪的装甲板槽位
     */
    int get_current_slot() const { return current_slot_; }

private:
    /**
     * @brief 处理装甲板切换
     */
    bool handle_armor_switch(const ArmorData& armor);

    /**
     * @brief 约束角速度
     */
    void constrain_omega();

    /**
     * @brief 构建过程噪声矩阵
     */
    MatrixXX build_Q(double dt) const;

    /**
     * @brief 构建观测噪声矩阵
     * @param distance 距离
     * @param z_to_v 装甲板朝向与视线夹角 (越大越侧面)
     */
    MatrixZZ build_R(double distance, double z_to_v) const;

    // EKF
    Ekf ekf_;
    bool initialized_ = false;
    double last_update_time_ = 0;

    // 槽位高度差 (通过观测学习，固定间隔10cm)
    std::array<double, 3> slot_dz_ = {0, 0, 0};    // 各槽位高度差
    std::array<bool, 3> slot_known_ = {false, false, false};  // 是否已观测到

    // 当前追踪
    int tracking_armor_id_ = -1;
    int current_slot_ = 0;          // 当前槽位 (0, 1, 2)
    double current_dz_ = 0;         // 当前装甲板高度差

    // 快速收敛：角速度方向判断
    bool omega_sign_determined_ = false;  // 角速度符号是否已确定
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_MOTION_OUTPOST_MOTION_MODEL_HPP__
