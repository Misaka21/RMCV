/**
 * @file sp_motion.hpp
 * @brief SP 整车旋转模型 - 移植自 sp_vision_25
 *
 * 与 SpinMotion/LmtdMotion 的关键区别:
 * - 状态包含 L (半径差 r_odd - r_even) 和 H (高度差 z_odd - z_even)
 * - 切板时不交换状态，而是根据 armor_id 选择使用哪个半径/高度
 * - 观测函数带 armor_id 参数
 *
 * 状态向量 (11维):
 *   [xc, vx, yc, vy, zc, vz, θ, ω, r, l, h]
 *   - xc, yc, zc: 旋转中心位置 (世界系)
 *   - vx, vy, vz: 旋转中心速度
 *   - θ: 车体朝向角 (OUTWARD, 从中心指向 id=0 装甲板, rad)
 *   - ω: 角速度 (rad/s)
 *   - r: 基础半径 (id=0,2 用)
 *   - l: 半径差 l = r_odd - r_even (id=1,3 用 r+l)
 *   - h: 高度差 h = z_odd - z_even (id=1,3 用 zc+h)
 *
 * 观测向量 (4维):
 *   [yaw, pitch, dis, armor_yaw]
 *   - yaw, pitch, dis: 装甲板位置的球坐标
 *   - armor_yaw: 装甲板朝向角 (= θ + id * 2π/N)
 *
 * 几何关系 (OUTWARD):
 *   armor_angle = θ + id * (2π / armor_num)
 *   xa = xc + r_actual * cos(armor_angle)
 *   ya = yc + r_actual * sin(armor_angle)
 *   za = z_actual
 *
 *   其中:
 *   use_l_h = (armor_num == 4) && (id == 1 || id == 3)
 *   r_actual = use_l_h ? (r + l) : r
 *   z_actual = use_l_h ? (zc + h) : zc
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_MOTION_SP_MOTION_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_MOTION_SP_MOTION_HPP__

#include <cmath>

#include <Eigen/Core>
#include <ceres/jet.h>

#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/auto_aim/predictor/observer/armor_tracker.hpp"
#include "aimer/common/filter/adaptive_ekf.hpp"
#include "motion_interface.hpp"

namespace autoaim::predictor {

// ============================================================================
// 常量定义
// ============================================================================

namespace sp_model {

constexpr int N_X = 11;  // 状态维度
constexpr int N_Z = 4;   // 观测维度

// 状态索引
enum StateIdx {
    XC = 0,          // 旋转中心 X
    VX = 1,          // X 速度
    YC = 2,          // 旋转中心 Y
    VY = 3,          // Y 速度
    ZC = 4,          // 旋转中心 Z
    VZ = 5,          // Z 速度
    THETA = 6,       // 车体朝向角 (OUTWARD, 指向 id=0 装甲板)
    OMEGA = 7,       // 角速度
    R = 8,           // 基础半径 (id=0,2 用)
    L = 9,           // 半径差 l = r_odd - r_even (id=1,3 用 r+l)
    H = 10           // 高度差 h = z_odd - z_even (id=1,3 用 zc+h)
};

// 观测索引
enum ObsIdx {
    YAW = 0,         // 装甲板方位角
    PITCH = 1,       // 装甲板俯仰角
    DIS = 2,         // 装甲板距离
    ARMOR_YAW = 3    // 装甲板朝向角
};

}  // namespace sp_model

// ============================================================================
// 预测函数 (CV模型，线性)
// ============================================================================

/**
 * @brief 匀速预测函数
 */
struct SpPredict {
    double dt;

    explicit SpPredict(double delta_t) : dt(delta_t) {}

    template<typename T>
    void operator()(const T x_in[sp_model::N_X], T x_out[sp_model::N_X]) const {
        // 中心位置 += 速度 * dt
        x_out[sp_model::XC] = x_in[sp_model::XC] + T(dt) * x_in[sp_model::VX];
        x_out[sp_model::VX] = x_in[sp_model::VX];

        x_out[sp_model::YC] = x_in[sp_model::YC] + T(dt) * x_in[sp_model::VY];
        x_out[sp_model::VY] = x_in[sp_model::VY];

        x_out[sp_model::ZC] = x_in[sp_model::ZC] + T(dt) * x_in[sp_model::VZ];
        x_out[sp_model::VZ] = x_in[sp_model::VZ];

        // 朝向角 += 角速度 * dt
        x_out[sp_model::THETA] = x_in[sp_model::THETA] + T(dt) * x_in[sp_model::OMEGA];
        x_out[sp_model::OMEGA] = x_in[sp_model::OMEGA];

        // r, l, h 不变
        x_out[sp_model::R] = x_in[sp_model::R];
        x_out[sp_model::L] = x_in[sp_model::L];
        x_out[sp_model::H] = x_in[sp_model::H];
    }
};

// ============================================================================
// 观测函数 (带 armor_id 参数)
// ============================================================================

/**
 * @brief 状态 → YPD 观测 (带 armor_id)
 *
 * 根据 armor_id 选择使用哪个半径和高度:
 * - id=0,2 (偶数): 使用 r, zc
 * - id=1,3 (奇数): 使用 r+l, zc+h
 *
 * @param armor_id 装甲板编号 (0-3)
 * @param armor_num 装甲板总数 (3 或 4)
 */
struct SpMeasure {
    int armor_id;
    int armor_num;

    SpMeasure(int id, int num) : armor_id(id), armor_num(num) {}

    template<typename T>
    void operator()(const T x[sp_model::N_X], T z[sp_model::N_Z]) const {
        // 计算装甲板角度
        T angle = x[sp_model::THETA] + T(armor_id * 2.0 * M_PI / armor_num);

        // 判断是否使用 l, h (4装甲板且奇数编号)
        bool use_l_h = (armor_num == 4) && (armor_id % 2 == 1);

        // 选择半径和高度
        T r_actual = use_l_h ? (x[sp_model::R] + x[sp_model::L]) : x[sp_model::R];
        T z_actual = use_l_h ? (x[sp_model::ZC] + x[sp_model::H]) : x[sp_model::ZC];

        // 计算装甲板位置 (OUTWARD: armor = center + r * (cos θ, sin θ))
        T xa = x[sp_model::XC] + r_actual * ceres::cos(angle);
        T ya = x[sp_model::YC] + r_actual * ceres::sin(angle);
        T za = z_actual;

        // 计算 YPD 观测
        T rho_sq = xa * xa + ya * ya;
        T rho = ceres::sqrt(rho_sq);
        T d = ceres::sqrt(rho_sq + za * za);

        z[sp_model::YAW] = ceres::atan2(ya, xa);
        z[sp_model::PITCH] = ceres::atan2(za, rho);
        z[sp_model::DIS] = d;
        z[sp_model::ARMOR_YAW] = angle;
    }
};

// ============================================================================
// SpMotion - 整车旋转模型 (SP Vision 风格)
// ============================================================================

/**
 * @brief SP 整车旋转模型
 *
 * 核心设计 (与 SpinMotion 的区别):
 * 1. 状态包含 l, h，不需要外部维护 another_r, another_dz
 * 2. 切板时不交换状态，而是根据 armor_id 选择
 * 3. 观测函数带 armor_id 参数
 *
 * 优点:
 * - 切板时状态连续，不会跳变
 * - 简化代码，减少 bug
 */
class SpMotion : public MotionInterface {
public:
    using Ekf = aimer::filter::AdaptiveEkf<sp_model::N_X, sp_model::N_Z>;
    using VectorX = Eigen::Matrix<double, sp_model::N_X, 1>;
    using VectorZ = Eigen::Matrix<double, sp_model::N_Z, 1>;
    using MatrixXX = Eigen::Matrix<double, sp_model::N_X, sp_model::N_X>;
    using MatrixZZ = Eigen::Matrix<double, sp_model::N_Z, sp_model::N_Z>;

    /**
     * @brief 构造
     * @param armor_num 装甲板数量 (3 或 4)
     */
    explicit SpMotion(int armor_num = 4);

    // ==================== MotionInterface 实现 ====================

    void init(const ArmorData& armor, double timestamp) override;
    void update(const ArmorData& armor, double timestamp) override;
    void update(const std::vector<ArmorData>& armors, double timestamp) override;
    void reset() override;
    bool valid() const override { return initialized_; }

    Eigen::Vector3d predict_center(double dt) const override;
    Eigen::Vector3d predict_armor_pos(int armor_idx, double dt) const override;

    Eigen::Vector3d get_velocity() const override;
    Eigen::Vector3d get_armor_pos() const override;
    double get_theta() const override;
    double get_omega() const override { return ekf_.get_x()[sp_model::OMEGA]; }
    double get_radius() const override { return ekf_.get_x()[sp_model::R]; }
    double get_another_radius() const override {
        VectorX x = ekf_.get_x();
        return x[sp_model::R] + x[sp_model::L];
    }
    double get_dz() const override { return ekf_.get_x()[sp_model::H]; }
    int get_tracked_id() const override { return tracked_armor_id_; }

    std::vector<Eigen::Vector3d> compute_all_armors(double dt = 0) const override;
    void log_state(const std::string& prefix) const override;
    const char* name() const override { return "sp"; }
    int armor_num() const override { return armor_num_; }

    // ==================== 额外方法 ====================

    /**
     * @brief 从观测装甲板反推所有装甲板位置 (兼容旧接口)
     * @deprecated 使用 compute_all_armors() 替代
     */
    std::vector<Eigen::Vector3d> compute_all_armors_from_observation(
        const Eigen::Vector3d& observed_pos,
        double observed_theta) const;

    VectorX get_state() const { return ekf_.get_x(); }

private:
    /**
     * @brief 匹配装甲板 ID
     *
     * 根据观测的装甲板位置和朝向，匹配最可能的装甲板编号 (0-3)
     *
     * @param armor 观测的装甲板
     * @return 匹配的装甲板编号
     */
    int match_armor(const ArmorData& armor) const;

    /**
     * @brief 辅助函数: 从状态计算指定装甲板位置
     */
    Eigen::Vector3d h_armor_xyz(const VectorX& x, int id) const;

    MatrixXX build_Q(double dt) const;
    MatrixZZ build_R(double distance, double z_to_v, int observed_armor_count = 1) const;

    // ==================== EKF ====================
    Ekf ekf_;
    bool initialized_ = false;
    double last_update_time_ = 0;

    // ==================== 装甲板配置 ====================
    int armor_num_ = 4;

    // ==================== 追踪状态 ====================
    int tracked_armor_id_ = 0;   // 当前追踪的 state_id (0-3)
    int last_detector_id_ = -1;  // 上一帧的 armor.id（ArmorIdentifier 分配的跟踪 ID）
};

// ============================================================================
// 辅助函数
// ============================================================================

/**
 * @brief 从状态计算装甲板位置 (指定编号)
 */
inline Eigen::Vector3d sp_state_to_armor_pos(const SpMotion::VectorX& state, int armor_id, int armor_num) {
    bool use_l_h = (armor_num == 4) && (armor_id % 2 == 1);
    double r_actual = use_l_h ? (state[sp_model::R] + state[sp_model::L]) : state[sp_model::R];
    double z_actual = use_l_h ? (state[sp_model::ZC] + state[sp_model::H]) : state[sp_model::ZC];
    double angle = state[sp_model::THETA] + armor_id * 2.0 * M_PI / armor_num;

    return Eigen::Vector3d(
        state[sp_model::XC] + r_actual * std::cos(angle),
        state[sp_model::YC] + r_actual * std::sin(angle),
        z_actual
    );
}

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_MOTION_SP_MOTION_HPP__
