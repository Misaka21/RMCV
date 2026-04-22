/**
 * @file sp_inekf.hpp
 * @brief SP 整车旋转模型 - InEKF 版本 (SE2 左不变)
 *
 * 与 SpMotion 的区别:
 * - 状态位姿 (xc, yc, theta) 放在 SE(2) 李群上，而非欧氏向量
 * - 预测步骤使用群上的指数映射，误差动力学自治
 * - 更新步骤通过李代数指数映射修正位姿，避免角度环绕
 * - 协方差定义在李代数切空间上，大机动下一致性更好
 *
 * 状态分解 (11维):
 *   群状态 X ∈ SE(2):  (xc, yc, theta)         -> 李代数误差 ξ ∈ R^3
 *   向量状态 b ∈ R^8:  [zc, vx, vy, vz, ω, r, l, h]
 *
 * 观测: YPD + armor_yaw (世界坐标系)，通过 Ceres Jet 在李代数上自动微分
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_MOTION_SP_INEKF_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_MOTION_SP_INEKF_HPP__

#include <cmath>

#include <Eigen/Core>
#include <ceres/jet.h>

#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/auto_aim/predictor/observer/armor_tracker.hpp"
#include "aimer/common/filter/invariant_ekf.hpp"
#include "aimer/common/math/se2.hpp"
#include "../ekf/motion_interface.hpp"

namespace autoaim::predictor {

namespace sp_inekf_model {

constexpr int VEC_DIM = 8;
constexpr int MEAS_DIM = 4;
constexpr int TOTAL_DIM = 3 + VEC_DIM;  // 11

// 向量状态索引 (b 中的位置)
enum VecIdx {
    ZC = 0,     ///< 旋转中心 Z
    VX = 1,     ///< X 速度 (世界坐标系)
    VY = 2,     ///< Y 速度
    VZ = 3,     ///< Z 速度
    OMEGA = 4,  ///< 角速度
    R = 5,      ///< 基础半径
    L = 6,      ///< 半径差
    H = 7       ///< 高度差
};

// 观测索引
enum MeasIdx {
    YAW = 0,
    PITCH = 1,
    DIS = 2,
    ARMOR_YAW = 3
};

} // namespace sp_inekf_model

// ============================================================================
// 观测函数 (Ceres Jet 自动微分)
// ============================================================================

/**
 * @brief InEKF 观测函数
 *
 * 输入 x[0:3] = 李代数误差 ξ, x[3:11] = 向量状态 b
 * 内部使用 exp(ξ) · X_nominal 构造扰动后的位姿
 */
struct SpInekfMeasure {
    double xc_nom;
    double yc_nom;
    double theta_nom;
    int armor_id;
    int armor_num;

    SpInekfMeasure(double xc_, double yc_, double th_, int id_, int num_)
        : xc_nom(xc_), yc_nom(yc_), theta_nom(th_), armor_id(id_), armor_num(num_) {}

    template<typename T>
    void operator()(const T x[sp_inekf_model::TOTAL_DIM], T z[sp_inekf_model::MEAS_DIM]) const {
        // ---- 1. 从李代数误差构造扰动后的 SE2: X_p = exp(ξ) · X_nom ----
        T xi_t = x[0];
        T xi_x = x[1];
        T xi_y = x[2];

        T ct = ceres::cos(xi_t), st = ceres::sin(xi_t);
        T V00, V01, V10, V11;
        if (ceres::abs(xi_t) < T(1e-8)) {
            V00 = T(1.0);            V01 = -T(0.5) * xi_t;
            V10 = T(0.5) * xi_t;     V11 = T(1.0);
        } else {
            V00 = st / xi_t;         V01 = (ct - T(1.0)) / xi_t;
            V10 = (T(1.0) - ct) / xi_t; V11 = V00;
        }
        T dx = V00 * xi_x + V01 * xi_y;
        T dy = V10 * xi_x + V11 * xi_y;

        T c0 = ceres::cos(T(theta_nom));
        T s0 = ceres::sin(T(theta_nom));
        T xc = T(xc_nom) + c0 * dx - s0 * dy;
        T yc = T(yc_nom) + s0 * dx + c0 * dy;
        T theta = T(theta_nom) + xi_t;

        // ---- 2. 向量状态 ----
        T zc     = x[3];
        // x[4]=vx, x[5]=vy, x[6]=vz, x[7]=omega, x[8]=r, x[9]=l, x[10]=h

        // ---- 3. 装甲板几何 ----
        bool use_l_h = (armor_num == 4) && (armor_id % 2 == 1);
        T r_actual = use_l_h ? (x[8] + x[9]) : x[8];
        T z_actual = use_l_h ? (zc + x[10]) : zc;
        T angle = theta + T(armor_id * 2.0 * M_PI / armor_num);

        // ---- 4. 世界坐标系装甲板位置 ----
        T xa = xc + r_actual * ceres::cos(angle);
        T ya = yc + r_actual * ceres::sin(angle);
        T za = z_actual;

        // ---- 5. YPD + armor_yaw ----
        T rho_sq = xa * xa + ya * ya;
        T rho = ceres::sqrt(rho_sq);
        T d = ceres::sqrt(rho_sq + za * za);

        z[sp_inekf_model::YAW]       = ceres::atan2(ya, xa);
        z[sp_inekf_model::PITCH]     = ceres::atan2(za, rho);
        z[sp_inekf_model::DIS]       = d;
        z[sp_inekf_model::ARMOR_YAW] = angle;
    }
};

// ============================================================================
// SpInekfMotion
// ============================================================================

class SpInekfMotion : public MotionInterface {
public:
    using Ekf = aimer::filter::InvariantEkf<sp_inekf_model::VEC_DIM, sp_inekf_model::MEAS_DIM>;
    using VectorB = Eigen::Matrix<double, sp_inekf_model::VEC_DIM, 1>;
    using VectorZ = Eigen::Matrix<double, sp_inekf_model::MEAS_DIM, 1>;
    using MatrixXX = Eigen::Matrix<double, sp_inekf_model::TOTAL_DIM, sp_inekf_model::TOTAL_DIM>;
    using MatrixZZ = Eigen::Matrix<double, sp_inekf_model::MEAS_DIM, sp_inekf_model::MEAS_DIM>;

    explicit SpInekfMotion(int armor_num = 4);

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
    double get_omega() const override;
    double get_radius() const override;
    double get_another_radius() const override;
    double get_dz() const override;
    int get_tracked_id() const override { return tracked_armor_id_; }

    std::vector<Eigen::Vector3d> compute_all_armors(double dt = 0) const override;
    void log_state(const std::string& prefix) const override;
    const char* name() const override { return "sp_inekf"; }
    int armor_num() const override { return armor_num_; }

private:
    int match_armor(const ArmorData& armor) const;
    Eigen::Vector3d h_armor_xyz(const aimer::math::SE2& X, const VectorB& b, int id) const;
    MatrixXX build_Q(double dt) const;
    MatrixZZ build_R(double distance, double z_to_v, int observed_armor_count = 1) const;

    Ekf ekf_;
    bool initialized_ = false;
    double last_update_time_ = 0;
    int armor_num_ = 4;
    int tracked_armor_id_ = 0;
    int last_detector_id_ = -1;
};

} // namespace autoaim::predictor

#endif // __AIMER_AUTO_AIM_PREDICTOR_MOTION_SP_INEKF_HPP__
