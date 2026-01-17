/**
 * @file lmtd_motion.hpp
 * @brief LMTD 整车旋转模型 - 参考 rm.cv.fans lmtd::TopModel
 *
 * 与 SpinMotion 的关键区别:
 * - x[4] = 装甲板 z (不是中心 z)，避免 PnP 的 z 误差累积
 * - 角度约定 OUTWARD: θ 从中心指向装甲板
 * - 装甲板位置: armor = center + r * (cos θ, sin θ)
 *
 * 状态向量 (9维):
 *   [xc, vx, yc, vy, za, vz, θ, ω, r]
 *   - xc, yc: 旋转中心位置 (世界系 XY)
 *   - vx, vy: 中心速度
 *   - za: 当前追踪装甲板的 z (不是中心 z!)
 *   - vz: 装甲板 z 速度 (通常强制为 0)
 *   - θ: 装甲板朝向角 (OUTWARD, 从中心指向装甲板, rad)
 *   - ω: 角速度 (rad/s)
 *   - r: 当前追踪装甲板半径
 *
 * 观测向量 (4维):
 *   [yaw, pitch, dis, orientation_yaw]
 *   - yaw, pitch, dis: 装甲板位置的球坐标
 *   - orientation_yaw: 装甲板朝向角 (= θ, OUTWARD)
 *
 * 几何关系 (OUTWARD):
 *   armor = center + r * (cos θ, sin θ)   (θ 从中心指向装甲板)
 *   center = armor - r * (cos θ, sin θ)   (从装甲板反推中心)
 *
 * 外部维护:
 *   - another_r: 另一个半径 (4装甲板车辆长短轴)
 *   - dz: 高度差 (当前装甲板与下一块装甲板的 z 差)
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_MOTION_LMTD_MOTION_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_MOTION_LMTD_MOTION_HPP__

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

namespace lmtd_model {

constexpr int N_X = 9;  // 状态维度
constexpr int N_Z = 4;  // 观测维度

// 状态索引
enum StateIdx {
    XC = 0,          // 旋转中心 X
    VX = 1,          // X 速度
    YC = 2,          // 旋转中心 Y
    VY = 3,          // Y 速度
    ZA = 4,          // 装甲板 Z (不是中心 z!)
    VZ = 5,          // Z 速度
    THETA = 6,       // 装甲板朝向角 (OUTWARD)
    OMEGA = 7,       // 角速度
    R = 8            // 当前半径
};

// 观测索引
enum ObsIdx {
    YAW = 0,         // 装甲板方位角
    PITCH = 1,       // 装甲板俯仰角
    DIS = 2,         // 装甲板距离
    ORIENT_YAW = 3   // 装甲板朝向角
};

}  // namespace lmtd_model

// ============================================================================
// 预测函数 (CV模型，线性)
// ============================================================================

/**
 * @brief 匀速预测函数
 */
struct LmtdPredict {
    double dt;

    explicit LmtdPredict(double delta_t) : dt(delta_t) {}

    template<typename T>
    void operator()(const T x_in[lmtd_model::N_X], T x_out[lmtd_model::N_X]) const {
        // 中心位置 += 速度 * dt
        x_out[lmtd_model::XC] = x_in[lmtd_model::XC] + T(dt) * x_in[lmtd_model::VX];
        x_out[lmtd_model::VX] = x_in[lmtd_model::VX];

        x_out[lmtd_model::YC] = x_in[lmtd_model::YC] + T(dt) * x_in[lmtd_model::VY];
        x_out[lmtd_model::VY] = x_in[lmtd_model::VY];

        // 装甲板 z += vz * dt
        x_out[lmtd_model::ZA] = x_in[lmtd_model::ZA] + T(dt) * x_in[lmtd_model::VZ];
        x_out[lmtd_model::VZ] = x_in[lmtd_model::VZ];

        // 朝向角 += 角速度 * dt
        x_out[lmtd_model::THETA] = x_in[lmtd_model::THETA] + T(dt) * x_in[lmtd_model::OMEGA];
        x_out[lmtd_model::OMEGA] = x_in[lmtd_model::OMEGA];

        // 半径不变
        x_out[lmtd_model::R] = x_in[lmtd_model::R];
    }
};

// ============================================================================
// 观测函数 (状态 → YPD观测)
// ============================================================================

/**
 * @brief 状态 → YPD 观测
 *
 * OUTWARD 约定: θ 从中心指向装甲板
 * armor = center + r * (cos θ, sin θ)
 *
 * 观测:
 *   yaw = atan2(ya, xa)
 *   pitch = atan2(za, √(xa² + ya²))
 *   dis = √(xa² + ya² + za²)
 *   orient_yaw = θ
 */
struct LmtdMeasure {
    template<typename T>
    void operator()(const T x[lmtd_model::N_X], T z[lmtd_model::N_Z]) const {
        // 从状态提取
        T xc = x[lmtd_model::XC];
        T yc = x[lmtd_model::YC];
        T za = x[lmtd_model::ZA];
        T theta = x[lmtd_model::THETA];
        T r = x[lmtd_model::R];

        // 计算装甲板位置 (OUTWARD: armor = center + r * (cos θ, sin θ))
        T xa = xc + r * ceres::cos(theta);
        T ya = yc + r * ceres::sin(theta);

        // 计算 YPD 观测
        T rho_sq = xa * xa + ya * ya;
        T rho = ceres::sqrt(rho_sq);
        T d = ceres::sqrt(rho_sq + za * za);

        z[lmtd_model::YAW] = ceres::atan2(ya, xa);
        z[lmtd_model::PITCH] = ceres::atan2(za, rho);
        z[lmtd_model::DIS] = d;
        z[lmtd_model::ORIENT_YAW] = theta;
    }
};

// ============================================================================
// LmtdMotion - 整车旋转模型
// ============================================================================

/**
 * @brief LMTD 整车旋转模型
 *
 * 关键设计 (与 SpinMotion 的区别):
 * 1. x[4] = 装甲板 z，不是中心 z
 *    - 避免 PnP 的 z 误差通过中心反推累积
 *    - 跳变时直接设 x[4] = 新装甲板 z
 *
 * 2. OUTWARD 约定: armor = center + r * (cos θ, sin θ)
 *
 * 3. dz 定义: 当前装甲板 z - 下一块装甲板 z
 *    - 跳变时: new_dz = old_za - new_za
 */
class LmtdMotion : public MotionInterface {
public:
    using Ekf = aimer::filter::AdaptiveEkf<lmtd_model::N_X, lmtd_model::N_Z>;
    using VectorX = Eigen::Matrix<double, lmtd_model::N_X, 1>;
    using VectorZ = Eigen::Matrix<double, lmtd_model::N_Z, 1>;
    using MatrixXX = Eigen::Matrix<double, lmtd_model::N_X, lmtd_model::N_X>;
    using MatrixZZ = Eigen::Matrix<double, lmtd_model::N_Z, lmtd_model::N_Z>;

    /**
     * @brief 构造
     * @param armor_num 装甲板数量 (3 或 4)
     */
    explicit LmtdMotion(int armor_num = 4);

    // ==================== MotionInterface 实现 ====================

    void init(const ArmorData& armor, double timestamp) override;
    void update(const ArmorData& armor, double timestamp) override;
    void update(const std::vector<ArmorData>& armors, double timestamp) override;
    void reset() override;
    bool valid() const override { return initialized_; }

    Eigen::Vector3d predict_center(double dt) const override;
    Eigen::Vector3d predict_armor_pos(int armor_idx, double dt) const override;

    Eigen::Vector3d get_velocity() const override { return get_center_velocity(); }
    Eigen::Vector3d get_armor_pos() const override;
    double get_theta() const override { return ekf_.get_x()[lmtd_model::THETA]; }
    double get_omega() const override { return ekf_.get_x()[lmtd_model::OMEGA]; }
    double get_radius() const override { return ekf_.get_x()[lmtd_model::R]; }
    double get_another_radius() const override { return another_r_; }
    double get_dz() const override { return dz_; }
    SpinLevel get_spin_level() const override { return spin_level_; }
    int get_tracked_id() const override { return tracked_armor_id_; }

    std::vector<Eigen::Vector3d> compute_all_armors(double dt = 0) const override;
    void output_to_plotter(const std::string& prefix) const override;
    const char* name() const override { return "lmtd"; }
    int armor_num() const override { return armor_num_; }

    // ==================== 额外方法 ====================

    /**
     * @brief 获取装甲板速度 (包含旋转分量)
     */
    Eigen::Vector3d get_armor_velocity() const;

    /**
     * @brief 获取旋转中心速度 (不含旋转分量)
     */
    Eigen::Vector3d get_center_velocity() const;

    /**
     * @brief 从观测装甲板反推所有装甲板位置 (兼容旧接口)
     * @deprecated 使用 compute_all_armors() 替代
     */
    std::vector<Eigen::Vector3d> compute_all_armors_from_observation(
        const Eigen::Vector3d& observed_pos,
        double observed_theta,
        int observed_id) const;

    /**
     * @brief 检查是否可信 (超时则不可信)
     */
    bool credit(double current_time) const;

    VectorX get_state() const { return ekf_.get_x(); }

private:
    /**
     * @brief 内部跳变检测和处理 (LMTD 核心 trick)
     *
     * 通过比较 tracked_armor_id 和当前观测的 armor.id 判断是否跳变
     * 如果跳变，计算跳了几块装甲板，交换半径/高度差
     *
     * @param armor 当前观测的装甲板
     * @param out_tracked_id [输出] 新的 tracked_armor_id (EKF 更新后才真正更新成员)
     * @return 是否发生跳变
     */
    bool detect_and_handle_jump(const ArmorData& armor, int& out_tracked_id);

    /**
     * @brief 选择要追踪的装甲板 (多装甲板时)
     *
     * 策略:
     * - 优先保持追踪当前 ID (防止反复横跳)
     * - 如果当前 ID 面积太小，切换到最大的
     */
    int select_armor_to_track(const std::vector<ArmorData>& armors) const;

    void update_spin_level();
    MatrixXX build_Q(double dt) const;
    MatrixZZ build_R(double distance, double z_to_v, int observed_armor_count = 1) const;

    // ==================== EKF ====================
    Ekf ekf_;
    bool initialized_ = false;
    double last_update_time_ = 0;
    double predict_t_ = 0;   // 上次预测时间
    double update_t_ = 0;    // 上次观测更新时间 (用于 credit 判断)

    // ==================== 装甲板配置 ====================
    int armor_num_ = 4;
    double another_r_ = 0.26;   // 另一个半径
    double dz_ = 0;             // 高度差 (当前 z - 下一块 z)

    // ==================== 追踪状态 ====================
    int tracked_armor_id_ = -1;  // 当前追踪的装甲板 ID (用于内部跳变检测)

    // ==================== 陀螺状态 ====================
    SpinLevel spin_level_ = SpinLevel::NONE;
    int top_level_ = 0;  // LMTD 风格: 0/1/2
};

// ============================================================================
// 辅助函数
// ============================================================================

/**
 * @brief 从状态计算装甲板位置
 */
inline Eigen::Vector3d lmtd_state_to_armor_pos(const LmtdMotion::VectorX& state) {
    return Eigen::Vector3d(
        state[lmtd_model::XC] + std::cos(state[lmtd_model::THETA]) * state[lmtd_model::R],
        state[lmtd_model::YC] + std::sin(state[lmtd_model::THETA]) * state[lmtd_model::R],
        state[lmtd_model::ZA]
    );
}

/**
 * @brief 从状态计算装甲板速度 (包含旋转切向速度)
 */
inline Eigen::Vector3d lmtd_state_to_armor_velocity(const LmtdMotion::VectorX& state) {
    double theta = state[lmtd_model::THETA];
    double omega = state[lmtd_model::OMEGA];
    double r = state[lmtd_model::R];

    // 切向速度: ω × r, 方向垂直于半径
    // θ 方向是 (cos θ, sin θ), 垂直方向是 (-sin θ, cos θ)
    double tangent_vx = -omega * r * std::sin(theta);
    double tangent_vy = omega * r * std::cos(theta);

    return Eigen::Vector3d(
        state[lmtd_model::VX] + tangent_vx,
        state[lmtd_model::VY] + tangent_vy,
        state[lmtd_model::VZ]
    );
}

/**
 * @brief 从状态计算中心速度
 */
inline Eigen::Vector3d lmtd_state_to_center_velocity(const LmtdMotion::VectorX& state) {
    return Eigen::Vector3d(
        state[lmtd_model::VX],
        state[lmtd_model::VY],
        state[lmtd_model::VZ]
    );
}

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_MOTION_LMTD_MOTION_HPP__
